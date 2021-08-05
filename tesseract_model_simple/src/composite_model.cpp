#include <tesseract_model_simple/composite_model.h>

namespace tesseract_model_simple
{
void setStateRecursive(CompositeModelState& model_state_set,
                       const VariableSets& variable_sets,
                       CompositeModelNode& node)
{
  node.model->setState(variable_sets);
  ModelState::UPtr ms_set = node.model->getState();

  if (node.parent != nullptr)
  {
    Eigen::Isometry3d parent_tf = model_state_set.getLinkTransformSets()[node.parent->getName()][node.parent_link];
    ms_set->setBaseTransform(parent_tf);
  }

  model_state_set.addModelState(std::move(ms_set));

  for (auto& child : node.children)
    setStateRecursive(model_state_set, variable_sets, *child);
}

void getStateRecursive(CompositeModelState& model_state_set,
                       const VariableSets& variable_sets,
                       CompositeModelNode& node)
{
  ModelState::UPtr ms_set = node.model->getState(variable_sets);

  if (node.parent != nullptr)
  {
    Eigen::Isometry3d parent_tf = model_state_set.getLinkTransformSets()[node.parent->getName()][node.parent_link];
    ms_set->setBaseTransform(parent_tf);
  }

  model_state_set.addModelState(std::move(ms_set));

  for (auto& child : node.children)
    getStateRecursive(model_state_set, variable_sets, *child);
}

void getLinkNamesRecursive(std::vector<std::string>& link_names, const CompositeModelNode& node)
{
  std::vector<std::string> child_link_names = node.model->getLinkNames();
  link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
  for (const auto& child : node.children)
    getLinkNamesRecursive(link_names, *child);
}

void getActiveLinkNamesRecursive(std::vector<std::string>& link_names, const CompositeModelNode& node, bool active)
{
  if (node.parent == nullptr)
  {
    std::vector<std::string> child_link_names = node.model->getActiveLinkNames();
    link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
    for (const auto& child : node.children)
      getActiveLinkNamesRecursive(link_names, *child, false);
  }
  else if (active)
  {
    std::vector<std::string> child_link_names = node.model->getLinkNames();
    link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
    for (const auto& child : node.children)
      getActiveLinkNamesRecursive(link_names, *child, true);
  }
  else
  {
    auto it = std::find(link_names.begin(), link_names.end(), node.parent_link);
    if (it == link_names.end())
    {
      std::vector<std::string> child_link_names = node.model->getActiveLinkNames();
      link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
      for (const auto& child : node.children)
        getActiveLinkNamesRecursive(link_names, *child, false);
    }
    else
    {
      std::vector<std::string> child_link_names = node.model->getLinkNames();
      link_names.insert(link_names.end(), child_link_names.begin(), child_link_names.end());
      for (const auto& child : node.children)
        getActiveLinkNamesRecursive(link_names, *child, true);
    }
  }
}

void getJointNamesRecursive(std::vector<std::string>& joint_names, const CompositeModelNode& node)
{
  std::vector<std::string> child_joint_names = node.model->getJointNames();
  joint_names.insert(joint_names.end(), child_joint_names.begin(), child_joint_names.end());
  for (const auto& child : node.children)
    getJointNamesRecursive(joint_names, *child);
}

void getActiveJointNamesRecursive(std::vector<std::string>& joint_names, const CompositeModelNode& node)
{
  std::vector<std::string> child_joint_names = node.model->getActiveJointNames();
  joint_names.insert(joint_names.end(), child_joint_names.begin(), child_joint_names.end());
  for (const auto& child : node.children)
    getActiveJointNamesRecursive(joint_names, *child);
}

CompositeModel::CompositeModel(std::string name, Model::UPtr root)
  : name_(std::move(name)), root_(std::make_unique<CompositeModelNode>())
{
  root_->model = root.get();
  nodes_[root->getName()] = root_.get();
  models_[root->getName()] = std::move(root);
}

void CompositeModel::addModel(std::string parent, std::string parent_link, Model::UPtr model)
{
  auto it = nodes_.find(parent);
  if (it == nodes_.end())
    throw std::runtime_error("Parent mode node not exist");

  auto child = std::make_unique<CompositeModelNode>();
  child->parent = it->second->model;
  child->parent_link = parent_link;
  child->model = model.get();

  nodes_[model->getName()] = child.get();
  it->second->children.push_back(std::move(child));
  models_[model->getName()] = std::move(model);
}

void recursiveClone(CompositeModel& model, const CompositeModelNode& node)
{
  model.addModel(node.parent->getName(), node.parent_link, node.model->clone());
  for (const auto& child : node.children)
    recursiveClone(model, *child);
}

Model::UPtr CompositeModel::clone() const
{
  auto cm = std::make_unique<CompositeModel>(name_, root_->model->clone());
  for (const auto& child : root_->children)
    recursiveClone(*cm, *child);

  return cm;
}

void CompositeModel::setState(const VariableSets& variable_sets)
{
  setStateRecursive(*current_state_, variable_sets, *root_);
}

ModelState::UPtr CompositeModel::getState() const { return current_state_->clone(); }

ModelState::UPtr CompositeModel::getState(const VariableSets& variable_sets) const
{
  auto cms = std::make_unique<CompositeModelState>();
  getStateRecursive(*cms, variable_sets, *root_);
  return cms;
}

std::string CompositeModel::getName() const { return name_; }
std::string CompositeModel::getRootLinkName() const { return root_->model->getRootLinkName(); }
std::vector<std::string> CompositeModel::getLinkNames() const
{
  std::vector<std::string> link_names;
  getLinkNamesRecursive(link_names, *root_);
  return link_names;
}
std::vector<std::string> CompositeModel::getActiveLinkNames() const
{
  std::vector<std::string> link_names;
  getActiveLinkNamesRecursive(link_names, *root_, false);
  return link_names;
}
std::vector<std::string> CompositeModel::getJointNames() const
{
  std::vector<std::string> joint_names;
  getJointNamesRecursive(joint_names, *root_);
  return joint_names;
}

std::vector<std::string> CompositeModel::getActiveJointNames() const
{
  std::vector<std::string> joint_names;
  getActiveJointNamesRecursive(joint_names, *root_);
  return joint_names;
}
}  // namespace tesseract_model_simple
