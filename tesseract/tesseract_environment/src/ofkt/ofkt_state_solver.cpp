/**
 * @file ofkt_state_solver.cpp
 * @brief A implementation of the Optimized Forward Kinematic Tree as a state solver.
 *
 * This is based on the paper "A Forward Kinematics Data Structure for Efficient Evolutionary Inverse Kinematics".
 *
 * @author Levi Armstrong
 * @date August 24, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_common/utils.h>

namespace tesseract_environment
{
class OFKTBaseNode : public OFKTNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OFKTBaseNode(tesseract_scene_graph::JointType type, OFKTNode* parent, std::string link_name)
    : type_(type), parent_(parent), link_name_(std::move(link_name))
  {
  }

  OFKTBaseNode(tesseract_scene_graph::JointType type,
               OFKTNode* parent,
               std::string link_name,
               std::string joint_name,
               Eigen::Isometry3d static_tf)
    : type_(type)
    , parent_(parent)
    , link_name_(std::move(link_name))
    , joint_name_(std::move(joint_name))
    , static_tf_(static_tf)
    , local_tf_(static_tf)
  {
  }

  tesseract_scene_graph::JointType getType() const override { return type_; }

  void setParent(OFKTNode* parent) override
  {
    parent_ = parent;
    update_world_required_ = true;
  }
  OFKTNode* getParent() override { return parent_; }
  const OFKTNode* getParent() const override { return parent_; }

  const std::string& getLinkName() const override { return link_name_; }
  const std::string& getJointName() const override { return joint_name_; }

  void storeJointValue(double joint_value) override
  {
    if (!tesseract_common::almostEqualRelativeAndAbs(joint_value_, joint_value, 1e-8))
    {
      joint_value_ = joint_value;
      joint_value_changed_ = true;
    }
  }

  double getJointValue() const override { return joint_value_; }

  bool hasJointValueChanged() const override { return joint_value_changed_; }

  void setStaticTransformation(Eigen::Isometry3d static_tf) override
  {
    static_tf_ = static_tf;
    local_tf_ = static_tf_ * joint_tf_;
    update_world_required_ = true;
  }

  const Eigen::Isometry3d& getStaticTransformation() const override { return static_tf_; }
  const Eigen::Isometry3d& getLocalTransformation() const override { return local_tf_; }
  Eigen::Isometry3d computeLocalTransformation(double /*joint_value*/) const override
  {
    throw std::runtime_error("OFKTBaseNode: computeLocalTransformation should never be called!");
  }

  void computeAndStoreWorldTransformation() override
  {
    world_tf_ = parent_->getWorldTransformation() * local_tf_;
    update_world_required_ = false;
  }
  const Eigen::Isometry3d& getWorldTransformation() const override { return world_tf_; }
  bool updateWorldTransformationRequired() const override { return update_world_required_; }

  void addChild(OFKTNode* node) override
  {
    children_.push_back(node);
    children_const_.push_back(node);
  }

  void removeChild(const OFKTNode* node) override
  {
    children_.erase(std::remove(children_.begin(), children_.end(), node));
    children_const_.erase(std::remove(children_const_.begin(), children_const_.end(), node));
  }

  std::vector<OFKTNode*>& getChildren() override { return children_; }

  const std::vector<const OFKTNode*>& getChildren() const override { return children_const_; }

protected:
  tesseract_scene_graph::JointType type_;
  OFKTNode* parent_{ nullptr };
  std::string link_name_;
  std::string joint_name_;
  Eigen::Isometry3d static_tf_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d joint_tf_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d local_tf_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d world_tf_{ Eigen::Isometry3d::Identity() };

  double joint_value_{ 0 };
  bool joint_value_changed_{ false };

  std::vector<OFKTNode*> children_;
  std::vector<const OFKTNode*> children_const_;

  bool update_world_required_{ true };

  friend class OFKTStateSolver;
};

class OFKTRootNode : public OFKTBaseNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief This should only be used for the root node of the tree
   * @param link_name The link name associated with the node
   */
  OFKTRootNode(std::string link_name)
    : OFKTBaseNode(tesseract_scene_graph::JointType::FIXED, nullptr, std::move(link_name))
  {
    update_world_required_ = false;
  }

  void setParent(OFKTNode* /*parent*/) override { throw std::runtime_error("OFKTRootNode: does not have a parent!"); }

  void storeJointValue(double /*joint_value*/) override
  {
    throw std::runtime_error("OFKTRootNode: does not have a joint value!");
  }

  void setStaticTransformation(Eigen::Isometry3d /*static_tf*/) override
  {
    throw std::runtime_error("OFKTRootNode: does not have a static transform!");
  }

  void computeAndStoreLocalTransformation() override { return; }
  void computeAndStoreWorldTransformation() override { return; }
  bool updateWorldTransformationRequired() const override { return false; }

  Eigen::Isometry3d computeLocalTransformation(double /*joint_value*/) const override { return static_tf_; }

private:
  friend class OFKTStateSolver;
};

class OFKTFixedNode : public OFKTBaseNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OFKTFixedNode(OFKTNode* parent, std::string link_name, std::string joint_name, Eigen::Isometry3d static_tf)
    : OFKTBaseNode(tesseract_scene_graph::JointType::FIXED,
                   parent,
                   std::move(link_name),
                   std::move(joint_name),
                   static_tf)
  {
    computeAndStoreWorldTransformation();
  }

  void storeJointValue(double /*joint_value*/) override
  {
    throw std::runtime_error("OFKTFixedNode: does not have a joint value!");
  }

  double getJointValue() const override { throw std::runtime_error("OFKTFixedNode: does not have a joint value!"); }

  void setStaticTransformation(Eigen::Isometry3d static_tf) override
  {
    static_tf_ = static_tf;
    local_tf_ = static_tf;
    update_world_required_ = true;
  }

  void computeAndStoreLocalTransformation() override { return; }

  Eigen::Isometry3d computeLocalTransformation(double /*joint_value*/) const override { return local_tf_; }

private:
  friend class OFKTStateSolver;
};

class OFKTRevoluteNode : public OFKTBaseNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OFKTRevoluteNode(OFKTNode* parent,
                   std::string link_name,
                   std::string joint_name,
                   Eigen::Isometry3d static_tf,
                   Eigen::Vector3d axis,
                   Eigen::Vector2d joint_limits)
    : OFKTBaseNode(tesseract_scene_graph::JointType::REVOLUTE,
                   parent,
                   std::move(link_name),
                   std::move(joint_name),
                   static_tf)
    , axis_(axis.normalized())
    , joint_limits_(joint_limits)
  {
    if (joint_limits_[1] < joint_limits_[0])
      throw std::runtime_error("OFKTRevoluteNode: Invalid joint limits!");

    if (tesseract_common::almostEqualRelativeAndAbs(joint_limits_[0], joint_limits_[1], 1e-5))
      throw std::runtime_error("OFKTRevoluteNode: Invalid joint limits!");

    computeAndStoreLocalTransformation();
    computeAndStoreWorldTransformation();
  }

  void storeJointValue(double joint_value) override
  {
    if (joint_value > joint_limits_[1])
    {
      CONSOLE_BRIDGE_logWarn("OFKTRevoluteNode: Joint value is above than upper limits, setting to upper limit.");
      joint_value = joint_limits_[1];
    }
    else if (joint_value < joint_limits_[0])
    {
      CONSOLE_BRIDGE_logWarn("OFKTRevoluteNode: Joint value is below the lower limits, setting to lower limit.");
      joint_value = joint_limits_[0];
    }

    OFKTBaseNode::storeJointValue(joint_value);
  }

  void computeAndStoreLocalTransformation() override
  {
    joint_tf_ = Eigen::AngleAxisd(joint_value_, axis_);
    local_tf_ = static_tf_ * joint_tf_;
    joint_value_changed_ = false;
  }

  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override
  {
    if (joint_value > joint_limits_[1])
    {
      CONSOLE_BRIDGE_logWarn("OFKTRevoluteNode: Joint value is above than upper limits, setting to upper limit.");
      joint_value = joint_limits_[1];
    }
    else if (joint_value < joint_limits_[0])
    {
      CONSOLE_BRIDGE_logWarn("OFKTRevoluteNode: Joint value is below the lower limits, setting to lower limit.");
      joint_value = joint_limits_[0];
    }

    return static_tf_ * Eigen::AngleAxisd(joint_value, axis_);
  }

  const Eigen::Vector3d& getAxis() const { return axis_; }

  const Eigen::Vector2d& getJointLimits() const { return joint_limits_; }

private:
  Eigen::Vector3d axis_;
  Eigen::Vector2d joint_limits_;

  friend class OFKTStateSolver;
};

class OFKTContinuousNode : public OFKTBaseNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OFKTContinuousNode(OFKTNode* parent,
                     std::string link_name,
                     std::string joint_name,
                     Eigen::Isometry3d static_tf,
                     Eigen::Vector3d axis)
    : OFKTBaseNode(tesseract_scene_graph::JointType::CONTINUOUS,
                   parent,
                   std::move(link_name),
                   std::move(joint_name),
                   static_tf)
    , axis_(axis.normalized())
  {
    computeAndStoreLocalTransformation();
    computeAndStoreWorldTransformation();
  }

  void computeAndStoreLocalTransformation() override
  {
    joint_tf_ = Eigen::AngleAxisd(joint_value_, axis_);
    local_tf_ = static_tf_ * joint_tf_;
    joint_value_changed_ = false;
  }

  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override
  {
    return static_tf_ * Eigen::AngleAxisd(joint_value, axis_);
  }

  const Eigen::Vector3d& getAxis() const { return axis_; }

private:
  Eigen::Vector3d axis_;

  friend class OFKTStateSolver;
};

class OFKTPrismaticNode : public OFKTBaseNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OFKTPrismaticNode(OFKTNode* parent,
                    std::string link_name,
                    std::string joint_name,
                    Eigen::Isometry3d static_tf,
                    Eigen::Vector3d axis)
    : OFKTBaseNode(tesseract_scene_graph::JointType::PRISMATIC,
                   parent,
                   std::move(link_name),
                   std::move(joint_name),
                   static_tf)
    , axis_(axis.normalized())
  {
    computeAndStoreLocalTransformation();
    computeAndStoreWorldTransformation();
  }

  void computeAndStoreLocalTransformation() override
  {
    joint_tf_ = Eigen::Translation3d(joint_value_ * axis_);
    local_tf_ = static_tf_ * joint_tf_;
    joint_value_changed_ = false;
  }

  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override
  {
    return static_tf_ * Eigen::Translation3d(joint_value * axis_);
  }

  const Eigen::Vector3d& getAxis() const { return axis_; }

private:
  Eigen::Vector3d axis_;

  friend class OFKTStateSolver;
};

/**
 * @brief Everytime a vertex is visited for the first time add a new
 *        segment to the KDL Tree;
 */
struct ofkt_builder : public boost::dfs_visitor<>
{
  ofkt_builder(OFKTStateSolver& tree,
               std::vector<std::pair<std::string, std::array<double, 2>>>& limits,
               std::string prefix = "")
    : tree_(tree), limits_(limits), prefix_(prefix)
  {
  }

  template <class u, class g>
  void discover_vertex(u vertex, g graph)
  {
    // Get incomming edges
    auto num_in_edges = static_cast<int>(boost::in_degree(vertex, graph));
    if (num_in_edges == 0)  // The root of the tree will have not incoming edges
      return;

    boost::graph_traits<tesseract_scene_graph::Graph>::in_edge_iterator ei, ei_end;
    boost::tie(ei, ei_end) = boost::in_edges(vertex, graph);
    tesseract_scene_graph::SceneGraph::Edge e = *ei;
    const tesseract_scene_graph::Joint::ConstPtr& joint = boost::get(boost::edge_joint, graph)[e];
    std::string joint_name = prefix_ + joint->getName();
    std::string parent_link_name = prefix_ + joint->parent_link_name;
    std::string child_link_name = prefix_ + joint->child_link_name;

    tree_.addNode(joint, joint_name, parent_link_name, child_link_name, limits_);
  }

protected:
  OFKTStateSolver& tree_;
  std::vector<std::pair<std::string, std::array<double, 2>>>& limits_;
  std::string prefix_;
};

void OFKTStateSolver::cloneHelper(OFKTStateSolver& cloned, const OFKTNode* node) const
{
  OFKTNode* parent_node = cloned.link_map_[node->getLinkName()];
  for (const OFKTNode* child : node->getChildren())
  {
    if (child->getType() == tesseract_scene_graph::JointType::FIXED)
    {
      auto n = std::make_unique<OFKTFixedNode>(
          parent_node, node->getLinkName(), node->getJointName(), node->getStaticTransformation());
      cloned.link_map_[node->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[node->getJointName()] = std::move(n);
    }
    else if (child->getType() == tesseract_scene_graph::JointType::REVOLUTE)
    {
      const auto* cn = static_cast<const OFKTRevoluteNode*>(node);

      auto n = std::make_unique<OFKTRevoluteNode>(parent_node,
                                                  node->getLinkName(),
                                                  node->getJointName(),
                                                  node->getStaticTransformation(),
                                                  cn->getAxis(),
                                                  cn->getJointLimits());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[node->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[node->getJointName()] = std::move(n);
    }
    else if (child->getType() == tesseract_scene_graph::JointType::CONTINUOUS)
    {
      const auto* cn = static_cast<const OFKTContinuousNode*>(node);

      auto n = std::make_unique<OFKTContinuousNode>(
          parent_node, node->getLinkName(), node->getJointName(), node->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[node->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[node->getJointName()] = std::move(n);
    }
    else if (child->getType() == tesseract_scene_graph::JointType::PRISMATIC)
    {
      const auto* cn = static_cast<const OFKTPrismaticNode*>(node);

      auto n = std::make_unique<OFKTPrismaticNode>(
          parent_node, node->getLinkName(), node->getJointName(), node->getStaticTransformation(), cn->getAxis());
      n->local_tf_ = cn->getLocalTransformation();
      n->world_tf_ = cn->getWorldTransformation();
      n->joint_value_ = cn->getJointValue();

      cloned.link_map_[node->getLinkName()] = n.get();
      parent_node->addChild(n.get());
      cloned.nodes_[node->getJointName()] = std::move(n);
    }
    else
    {
      throw std::runtime_error("Unsupport OFKTNode type!");
    }

    cloneHelper(cloned, child);
  }
}

StateSolver::Ptr OFKTStateSolver::clone() const
{
  auto cloned = std::make_shared<OFKTStateSolver>();
  cloned->root_ = std::make_unique<OFKTRootNode>(root_->getLinkName());
  cloned->link_map_[root_->getLinkName()] = cloned->root_.get();
  cloned->current_state_ = std::make_shared<EnvState>(*current_state_);
  cloned->joint_names_ = joint_names_;
  cloned->limits_ = limits_;
  return cloned;
}

bool OFKTStateSolver::init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph)
{
  assert(scene_graph->isTree());

  const std::string& root_name = scene_graph->getRoot();

  root_ = std::make_unique<OFKTRootNode>(root_name);
  link_map_[root_name] = root_.get();
  current_state_->link_transforms[root_name] = root_->getWorldTransformation();

  std::vector<std::pair<std::string, std::array<double, 2>>> limits;
  limits.resize(scene_graph->getJoints().size());
  ofkt_builder builder(*this, limits);

  std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t> index_map;
  boost::associative_property_map<std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t>> prop_index_map(
      index_map);

  int c = 0;
  tesseract_scene_graph::Graph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(*scene_graph); i != iend; ++i, ++c)
    boost::put(prop_index_map, *i, c);

  boost::depth_first_search(
      static_cast<const tesseract_scene_graph::Graph&>(*scene_graph),
      boost::visitor(builder).root_vertex(scene_graph->getVertex(root_name)).vertex_index_map(prop_index_map));

  // Populate Joint Limits
  limits_.resize(static_cast<long>(limits.size()), 2);
  for (std::size_t i = 0; i < limits.size(); ++i)
  {
    limits_(static_cast<long>(i), 0) = limits[i].second[0];
    limits_(static_cast<long>(i), 1) = limits[i].second[1];
  }

  revision_ = 1;

  update(root_.get(), false);

  return true;
}

void OFKTStateSolver::setState(const std::unordered_map<std::string, double>& joints)
{
  for (const auto& joint : joints)
  {
    nodes_[joint.first]->storeJointValue(joint.second);
    current_state_->joints[joint.first] = joint.second;
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values)
{
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    nodes_[joint_names[i]]->storeJointValue(joint_values[i]);
    current_state_->joints[joint_names[i]] = joint_values[i];
  }

  update(root_.get(), false);
}

void OFKTStateSolver::setState(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    nodes_[joint_names[i]]->storeJointValue(joint_values(static_cast<long>(i)));
    current_state_->joints[joint_names[i]] = joint_values(static_cast<long>(i));
  }

  update(root_.get(), false);
}

EnvState::Ptr OFKTStateSolver::getState(const std::unordered_map<std::string, double>& joints) const
{
  auto state = std::make_shared<EnvState>(*current_state_);
  for (const auto& joint : joints)
    state->joints[joint.first] = joint.second;

  update(*state, root_.get(), Eigen::Isometry3d::Identity(), false);
  return state;
}

EnvState::Ptr OFKTStateSolver::getState(const std::vector<std::string>& joint_names,
                                        const std::vector<double>& joint_values) const
{
  auto state = std::make_shared<EnvState>(*current_state_);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    state->joints[joint_names[i]] = joint_values[i];

  update(*state, root_.get(), Eigen::Isometry3d::Identity(), false);
  return state;
}

EnvState::Ptr OFKTStateSolver::getState(const std::vector<std::string>& joint_names,
                                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  auto state = std::make_shared<EnvState>(*current_state_);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    state->joints[joint_names[i]] = joint_values[static_cast<long>(i)];

  update(*state, root_.get(), Eigen::Isometry3d::Identity(), false);
  return state;
}

EnvState::ConstPtr OFKTStateSolver::getCurrentState() const { return current_state_; }

EnvState::Ptr OFKTStateSolver::getRandomState() const
{
  return getState(joint_names_, tesseract_common::generateRandomNumber(joint_limits_));
}

const Eigen::MatrixX2d& OFKTStateSolver::getLimits() const { return limits_; }

void OFKTStateSolver::onEnvironmentChanged(const Commands& commands)
{
  std::vector<std::pair<std::string, std::array<double, 2>>> new_limits;
  new_limits.reserve(commands.size());

  std::vector<std::string> removed_joints;
  removed_joints.reserve(commands.size());

  std::vector<long> removed_joints_indices;
  removed_joints_indices.reserve(commands.size());

  for (auto it = commands.begin() + revision_; it != commands.end(); ++it)
  {
    const Command::ConstPtr& command = *it;
    if (!command)
      throw std::runtime_error("OFKTStateSolver: Commands constains nullptr's");

    switch (command->getType())
    {
      case tesseract_environment::CommandType::ADD:
      {
        const auto& cmd = static_cast<const tesseract_environment::AddCommand&>(*command);
        const tesseract_scene_graph::Joint::ConstPtr& joint = cmd.getJoint();

        addNode(joint, joint->getName(), joint->parent_link_name, joint->child_link_name, new_limits);
        break;
      }
      case tesseract_environment::CommandType::MOVE_LINK:
      {
        const auto& cmd = static_cast<const tesseract_environment::MoveLinkCommand&>(*command);
        const tesseract_scene_graph::Joint::ConstPtr& joint = cmd.getJoint();

        auto* old_node = link_map_[cmd.getJoint()->child_link_name];
        const std::string& old_joint_name = old_node->getJointName();
        old_node->getParent()->removeChild(old_node);

        // Store the remove joints to process later
        auto it = std::find(joint_names_.begin(), joint_names_.end(), old_joint_name);
        if (it != joint_names_.end())
        {
          removed_joints.push_back(old_joint_name);
          removed_joints_indices.push_back(std::distance(joint_names_.begin(), it));
        }

        current_state_->joints.erase(old_joint_name);
        current_state_->joint_transforms.erase(old_joint_name);

        nodes_.erase(old_joint_name);

        addNode(joint, joint->getName(), joint->parent_link_name, joint->child_link_name, new_limits);
        break;
      }
      case tesseract_environment::CommandType::MOVE_JOINT:
      {
        const auto& cmd = static_cast<const tesseract_environment::MoveJointCommand&>(*command);
        auto& n = nodes_[cmd.getJointName()];
        n->getParent()->removeChild(n.get());
        OFKTNode* new_parent = link_map_[cmd.getParentLink()];
        n->setParent(new_parent);
        new_parent->addChild(n.get());
        break;
      }
      case tesseract_environment::CommandType::REMOVE_LINK:
      {
        const auto& cmd = static_cast<const tesseract_environment::RemoveLinkCommand&>(*command);
        auto* node = link_map_[cmd.getLinkName()];
        removeNode(node, removed_joints, removed_joints_indices);
        break;
      }
      case tesseract_environment::CommandType::REMOVE_JOINT:
      {
        const auto& cmd = static_cast<const tesseract_environment::RemoveJointCommand&>(*command);
        auto* node = nodes_[cmd.getJointName()].get();
        removeNode(node, removed_joints, removed_joints_indices);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
      {
        throw std::runtime_error("OFKTStateSolver: Environment command change link origin is not supported!");
      }
      case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
      {
        const auto& cmd = static_cast<const tesseract_environment::ChangeJointOriginCommand&>(*command);
        auto& n = nodes_[cmd.getJointName()];
        n->setStaticTransformation(cmd.getOrigin());
        break;
      }
      case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
      case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
      case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
      case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
      case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
      {
        break;
      }
      case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
      {
        const auto& cmd = static_cast<const tesseract_environment::AddSceneGraphCommand&>(*command);
        const tesseract_scene_graph::Joint::ConstPtr& joint = cmd.getJoint();

        addNode(joint, joint->getName(), joint->parent_link_name, joint->child_link_name, new_limits);

        ofkt_builder builder(*this, new_limits, cmd.getPrefix());

        std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t> index_map;
        boost::associative_property_map<std::map<tesseract_scene_graph::SceneGraph::Vertex, size_t>> prop_index_map(
            index_map);

        int c = 0;
        tesseract_scene_graph::Graph::vertex_iterator i, iend;
        for (boost::tie(i, iend) = boost::vertices(*cmd.getSceneGraph()); i != iend; ++i, ++c)
          boost::put(prop_index_map, *i, c);

        boost::depth_first_search(static_cast<const tesseract_scene_graph::Graph&>(*cmd.getSceneGraph()),
                                  boost::visitor(builder)
                                      .root_vertex(cmd.getSceneGraph()->getVertex(cmd.getSceneGraph()->getRoot()))
                                      .vertex_index_map(prop_index_map));

        break;
      }
      default:
      {
        throw std::runtime_error("OFKTStateSolver: Unhandled environment command");
      }
    }
    revision_ = static_cast<int>(commands.size());
  }

  // Remove deleted joints
  if (!removed_joints.empty())
  {
    joint_names_.erase(
        std::remove_if(joint_names_.begin(), joint_names_.end(), [removed_joints](const std::string& joint_name) {
          return (std::find(removed_joints.begin(), removed_joints.end(), joint_name) != removed_joints.end());
        }));
    Eigen::MatrixX2d l1(limits_.rows() - static_cast<long>(removed_joints.size()), 2);
    long cnt = 0;
    for (long i = 0; i < limits_.rows(); ++i)
      if (std::find(removed_joints_indices.begin(), removed_joints_indices.end(), i) == removed_joints_indices.end())
        l1.row(cnt++) = limits_.row(i);

    limits_ = l1;
  }

  // Populate Joint Limits
  if (!new_limits.empty())
  {
    Eigen::MatrixX2d l(limits_.rows() + static_cast<long>(new_limits.size()), 2);
    Eigen::MatrixX2d l2(static_cast<long>(new_limits.size()), 2);
    for (std::size_t i = 0; i < new_limits.size(); ++i)
    {
      l2(static_cast<long>(i), 0) = new_limits[i].second[0];
      l2(static_cast<long>(i), 1) = new_limits[i].second[1];
    }
    l << limits_, l2;
    limits_ = l;
  }

  update(root_.get(), false);
}

void OFKTStateSolver::update(OFKTNode* node, bool update_required)
{
  if (node->hasJointValueChanged())
  {
    node->computeAndStoreLocalTransformation();
    update_required = true;
  }

  if (node->updateWorldTransformationRequired())
    update_required = true;

  if (update_required)
  {
    node->computeAndStoreWorldTransformation();
    current_state_->link_transforms[node->getLinkName()] = node->getWorldTransformation();
    current_state_->joint_transforms[node->getJointName()] = node->getWorldTransformation();
  }

  for (auto* child : node->getChildren())
    update(child, update_required);
}

void OFKTStateSolver::update(EnvState& state,
                             const OFKTNode* node,
                             Eigen::Isometry3d parent_world_tf,
                             bool update_required) const
{
  if (node->getType() != tesseract_scene_graph::JointType::FIXED)
  {
    double jv = state.joints[node->getJointName()];
    if (!tesseract_common::almostEqualRelativeAndAbs(node->getJointValue(), jv, 1e-8))
    {
      parent_world_tf = parent_world_tf * node->computeLocalTransformation(jv);
      update_required = true;
    }
    else
    {
      parent_world_tf = parent_world_tf * node->getLocalTransformation();
    }
  }
  else
  {
    parent_world_tf = parent_world_tf * node->getLocalTransformation();
  }

  if (update_required)
  {
    state.link_transforms[node->getLinkName()] = parent_world_tf;
    state.joint_transforms[node->getJointName()] = parent_world_tf;
  }

  for (const auto* child : node->getChildren())
    update(state, child, parent_world_tf, update_required);
}

void OFKTStateSolver::addNode(const tesseract_scene_graph::Joint::ConstPtr& joint,
                              const std::string& joint_name,
                              const std::string& parent_link_name,
                              const std::string& child_link_name,
                              std::vector<std::pair<std::string, std::array<double, 2>>>& limits)
{
  OFKTNode::UPtr n;
  switch (joint->type)
  {
    case tesseract_scene_graph::JointType::FIXED:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      auto n = std::make_unique<OFKTFixedNode>(
          parent_node, child_link_name, joint_name, joint->parent_to_joint_origin_transform);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_->link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_->joint_transforms[n->getJointName()] = n->getWorldTransformation();
      nodes_[joint_name] = std::move(n);

      break;
    }
    case tesseract_scene_graph::JointType::REVOLUTE:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      auto n = std::make_unique<OFKTRevoluteNode>(parent_node,
                                                  child_link_name,
                                                  joint_name,
                                                  joint->parent_to_joint_origin_transform,
                                                  joint->axis,
                                                  Eigen::Vector2d(joint->limits->lower, joint->limits->upper));
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_->joints[joint_name] = 0;
      current_state_->link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_->joint_transforms[n->getJointName()] = n->getWorldTransformation();
      nodes_[joint_name] = std::move(n);
      joint_names_.push_back(joint_name);
      limits.push_back(
          std::make_pair(joint_name, std::array<double, 2>({ joint->limits->lower, joint->limits->upper })));
      break;
    }
    case tesseract_scene_graph::JointType::CONTINUOUS:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      auto n = std::make_unique<OFKTContinuousNode>(
          parent_node, child_link_name, joint_name, joint->parent_to_joint_origin_transform, joint->axis);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_->joints[joint_name] = 0;
      current_state_->link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_->joint_transforms[n->getJointName()] = n->getWorldTransformation();
      nodes_[joint_name] = std::move(n);
      joint_names_.push_back(joint_name);
      limits.push_back(
          std::make_pair(joint_name, std::array<double, 2>({ joint->limits->lower, joint->limits->upper })));
      break;
    }
    case tesseract_scene_graph::JointType::PRISMATIC:
    {
      OFKTNode* parent_node = link_map_[parent_link_name];
      auto n = std::make_unique<OFKTPrismaticNode>(
          parent_node, child_link_name, joint_name, joint->parent_to_joint_origin_transform, joint->axis);
      link_map_[child_link_name] = n.get();
      parent_node->addChild(n.get());
      current_state_->joints[joint_name] = 0;
      current_state_->link_transforms[n->getLinkName()] = n->getWorldTransformation();
      current_state_->joint_transforms[n->getJointName()] = n->getWorldTransformation();
      nodes_[joint_name] = std::move(n);
      joint_names_.push_back(joint_name);
      limits.push_back(
          std::make_pair(joint_name, std::array<double, 2>({ joint->limits->lower, joint->limits->upper })));
      break;
    }
    default:
    {
      throw std::runtime_error("Unsupported joint type for joint '" + joint_name + "'");
    }
  }
}

void OFKTStateSolver::removeNode(OFKTNode* node,
                                 std::vector<std::string>& removed_joints,
                                 std::vector<long>& removed_joints_indices)
{
  auto it = std::find(joint_names_.begin(), joint_names_.end(), node->getJointName());
  if (it != joint_names_.end())
  {
    removed_joints.push_back(node->getJointName());
    removed_joints_indices.push_back(std::distance(joint_names_.begin(), it));
  }

  current_state_->link_transforms.erase(node->getLinkName());
  current_state_->joints.erase(node->getJointName());
  current_state_->joint_transforms.erase(node->getJointName());

  std::vector<OFKTNode*> children = node->getChildren();
  for (auto* child : node->getChildren())
    removeNode(child, removed_joints, removed_joints_indices);

  if (node->getParent() != nullptr)
    node->getParent()->removeChild(node);

  link_map_.erase(node->getLinkName());
  nodes_.erase(node->getJointName());
}

}  // namespace tesseract_environment
