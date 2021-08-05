#include <tesseract_model_linked/model.h>

namespace tesseract_model_linked
{
void getModelActiveLinkNamesRecursive(std::vector<std::string>& active_links,
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
      getModelActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
  else
  {
    for (const auto& child_link : scene_graph.getAdjacentLinkNames(current_link))
      if (scene_graph.getInboundJoints(child_link)[0]->type != tesseract_scene_graph::JointType::FIXED)
        getModelActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
      else
        getModelActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
}

Model::Model(tesseract_scene_graph::SceneGraph::UPtr scene_graph)
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
  getModelActiveLinkNamesRecursive(active_link_names_, *scene_graph_, scene_graph_->getRoot(), false);

  // If scene graph is tree create default state solver
  if (scene_graph_->isTree())
  {
    state_solver_ = std::make_unique<ModelStateSolver>();
    state_solver_->init(*scene_graph_);
  }
}

void Model::setParent(Model* parent_model, std::string parent_link_name)
{
  parent_model_ = parent_model;
  parent_link_name_ = parent_link_name;
}

Model::Parent Model::getParent() const { return std::make_pair(parent_model_, parent_link_name_); }

void Model::addChild(Model::UPtr child_model, std::string attach_link_name)
{
  children_.push_back(std::make_pair(std::move(child_model), attach_link_name));
}

Model::Children& Model::getChildren() { return children_; }
const Model::Children& Model::getChildren() const { return children_; }

Model::UPtr Model::clone() const { return std::make_unique<Model>(scene_graph_->clone()); }

void Model::setState(const VariableSets& variable_set)
{
  auto it = variable_set.find(getName());
  if (it == variable_set.end())
    return;

  state_solver_->setState(it->second);
}

ModelState::UPtr Model::getState() const { return current_state_->clone(); }

ModelState::UPtr Model::getState(const VariableSets& variable_sets) const
{
  auto it = variable_sets.find(getName());
  if (it == variable_sets.end())
    return current_state_->clone();

  return state_solver_->getState(it->second);
}

std::string Model::getName() const { return scene_graph_->getName(); }
std::string Model::getRootLinkName() const { return scene_graph_->getRoot(); }
std::vector<std::string> Model::getLinkNames(bool recursive) const
{
  if (!recursive)
    return link_names_;

  std::vector<std::string> link_names(link_names_);
  for (const auto& child : children_)
  {
    std::vector<std::string> child_link_names = child.first->getLinkNames(true);
    link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
  }

  return link_names;
}

std::vector<std::string> Model::getActiveLinkNames(bool recursive) const
{
  if (!recursive)
    return active_link_names_;

  std::vector<std::string> link_names(active_link_names_);
  for (const auto& child : children_)
  {
    std::vector<std::string> child_link_names;
    if (std::find(active_link_names_.begin(), active_link_names_.end(), child.second) != active_link_names_.end())
      child_link_names = child.first->getLinkNames(true);
    else
      child_link_names = child.first->getActiveLinkNames(true);

    link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
  }

  return link_names;
}

std::vector<std::string> Model::getJointNames(bool recursive) const
{
  if (!recursive)
    return joint_names_;

  std::vector<std::string> joint_names(joint_names_);
  for (const auto& child : children_)
  {
    std::vector<std::string> child_joint_names = child.first->getJointNames(true);
    joint_names.insert(joint_names.end(), child_joint_names.begin(), child_joint_names.end());
  }

  return joint_names;
}

std::vector<std::string> Model::getActiveJointNames(bool recursive) const
{
  if (!recursive)
    return active_joint_names_;

  std::vector<std::string> joint_names(active_joint_names_);
  for (const auto& child : children_)
  {
    std::vector<std::string> child_joint_names = child.first->getActiveJointNames(true);
    joint_names.insert(joint_names.end(), child_joint_names.begin(), child_joint_names.end());
  }

  return joint_names;
}

}  // namespace tesseract_model_linked
