#include <tesseract_model_simple/scene_model.h>

namespace tesseract_model_simple
{
void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                 const tesseract_scene_graph::SceneGraph& scene_graph,
                                 const std::string& current_link,
                                 bool active)
{
  // recursively get all active links
  assert(scene_graph.isTree());
  if (active)
  {
    active_links.push_back(current_link);
    for (const auto& child_link : scene_graph.getAdjacentLinkNames(current_link))
      getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
  else
  {
    for (const auto& child_link : scene_graph.getAdjacentLinkNames(current_link))
      if (scene_graph.getInboundJoints(child_link)[0]->type != tesseract_scene_graph::JointType::FIXED)
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
}

SceneModel::SceneModel(tesseract_scene_graph::SceneGraph::UPtr scene_graph)
{
  scene_graph_ = std::move(scene_graph);

  // Update link names
  std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph_->getLinks();
  link_names_.clear();
  link_names_.reserve(links.size());
  for (const auto& link : links)
    link_names_.push_back(link->getName());

  // Update joint names and active joint name
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getJoints();
  active_joint_names_.clear();
  joint_names_.clear();
  joint_names_.reserve(joints.size());
  for (const auto& joint : joints)
  {
    joint_names_.push_back(joint->getName());

    // UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    if (joint->type == tesseract_scene_graph::JointType::REVOLUTE ||
        joint->type == tesseract_scene_graph::JointType::CONTINUOUS ||
        joint->type == tesseract_scene_graph::JointType::PRISMATIC)
      active_joint_names_.push_back(joint->getName());
  }

  // Update active link names
  active_link_names_.clear();
  getActiveLinkNamesRecursive(active_link_names_, *scene_graph_, scene_graph_->getRoot(), false);

  // If scene graph is tree create default state solver
  if (scene_graph_->isTree())
  {
    state_solver_ = std::make_unique<StateSolver>();
    state_solver_->init(*scene_graph_);
  }
}

Model::UPtr SceneModel::clone() const { return std::make_unique<SceneModel>(scene_graph_->clone()); }

void SceneModel::setState(const VariableSets& variable_set)
{
  auto it = variable_set.find(getName());
  if (it == variable_set.end())
    return;

  state_solver_->setState(it->second);
}

ModelState::UPtr SceneModel::getState() const { return current_state_->clone(); }

ModelState::UPtr SceneModel::getState(const VariableSets& variable_sets) const
{
  auto it = variable_sets.find(getName());
  if (it == variable_sets.end())
    return current_state_->clone();

  return state_solver_->getState(it->second);
}

std::string SceneModel::getName() const { return scene_graph_->getName(); }
std::string SceneModel::getRootLinkName() const { return scene_graph_->getRoot(); }
std::vector<std::string> SceneModel::getLinkNames() const { return link_names_; }
std::vector<std::string> SceneModel::getActiveLinkNames() const { return active_link_names_; }
std::vector<std::string> SceneModel::getJointNames() const { return joint_names_; }
std::vector<std::string> SceneModel::getActiveJointNames() const { return active_joint_names_; }
}  // namespace tesseract_model_simple
