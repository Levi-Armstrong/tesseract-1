#include <tesseract_model_simple/state_solver.h>
#include <tesseract_scene_graph/parser/kdl_parser.h>

namespace tesseract_model_simple
{
StateSolver::UPtr StateSolver::clone() const
{
  auto cloned_solver = std::make_unique<StateSolver>();
  cloned_solver->root_name_ = root_name_;
  cloned_solver->kdl_tree_ = kdl_tree_;
  cloned_solver->joint_to_qnr_ = joint_to_qnr_;
  cloned_solver->kdl_jnt_array_ = kdl_jnt_array_;
  return cloned_solver;
}

bool StateSolver::init(const tesseract_scene_graph::SceneGraph& scene_graph) { return createKDETree(scene_graph); }

void StateSolver::setState(const VariableSet& variable_set)
{
  for (auto& joint : variable_set)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
    {
      current_state_->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(*current_state_, kdl_jnt_array_, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());
}

ModelState::UPtr StateSolver::getState() const { return current_state_->clone(); }

ModelState::UPtr StateSolver::getState(const VariableSet& variable_set) const
{
  auto state = std::make_unique<SceneModelState>();
  state->name = current_state_->name;
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto& joint : variable_set)
  {
    if (setJointValuesHelper(jnt_array, joint.first, joint.second))
    {
      state->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(*state, jnt_array, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());

  return state;
}

bool StateSolver::createKDETree(const tesseract_scene_graph::SceneGraph& scene_graph)
{
  root_name_ = scene_graph.getRoot();
  kdl_tree_ = KDL::Tree();
  if (!tesseract_scene_graph::parseSceneGraph(scene_graph, kdl_tree_))
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  current_state_ = std::make_unique<SceneModelState>();
  current_state_->name = scene_graph.getName();
  kdl_jnt_array_.resize(kdl_tree_.getNrOfJoints());
  //  limits_.joint_limits.resize(static_cast<long int>(kdl_tree_.getNrOfJoints()), 2);
  //  limits_.velocity_limits.resize(static_cast<long int>(kdl_tree_.getNrOfJoints()));
  //  limits_.acceleration_limits.resize(static_cast<long int>(kdl_tree_.getNrOfJoints()));
  //  joint_names_.resize(kdl_tree_.getNrOfJoints());
  joint_to_qnr_.clear();
  size_t j = 0;
  for (const auto& seg : kdl_tree_.getSegments())
  {
    const KDL::Joint& jnt = seg.second.segment.getJoint();

    if (jnt.getType() == KDL::Joint::None)
      continue;

    joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
    kdl_jnt_array_(seg.second.q_nr) = 0.0;
    current_state_->joints.insert(std::make_pair(jnt.getName(), 0.0));
    //    joint_names_[j] = jnt.getName();

    //    // Store joint limits.
    //    const auto& sj = scene_graph_->getJoint(jnt.getName());
    //    limits_.joint_limits(static_cast<long>(j), 0) = sj->limits->lower;
    //    limits_.joint_limits(static_cast<long>(j), 1) = sj->limits->upper;
    //    limits_.velocity_limits(static_cast<long>(j)) = sj->limits->velocity;
    //    limits_.acceleration_limits(static_cast<long>(j)) = sj->limits->acceleration;

    j++;
  }

  calculateTransforms(*current_state_, kdl_jnt_array_, kdl_tree_.getRootSegment(), Eigen::Isometry3d::Identity());
  return true;
}

bool StateSolver::setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const
{
  auto qnr = joint_to_qnr_.find(joint_name);
  if (qnr != joint_to_qnr_.end())
  {
    q(qnr->second) = joint_value;
    return true;
  }

  CONSOLE_BRIDGE_logError("Tried to set joint name %s which does not exist!", joint_name.c_str());
  return false;
}

void KDLToEigen(const KDL::Frame& frame, Eigen::Isometry3d& transform)
{
  transform.setIdentity();

  // translation
  for (int i = 0; i < 3; ++i)
    transform(i, 3) = frame.p[i];

  // rotation matrix
  for (int i = 0; i < 9; ++i)
    transform(i / 3, i % 3) = frame.M.data[i];
}

void StateSolver::calculateTransformsHelper(SceneModelState& state,
                                            const KDL::JntArray& q_in,
                                            const KDL::SegmentMap::const_iterator& it,
                                            const Eigen::Isometry3d& parent_frame) const
{
  if (it != kdl_tree_.getSegments().end())
  {
    const KDL::TreeElementType& current_element = it->second;
    KDL::Frame current_frame;
    if (q_in.data.size() > 0)
      current_frame = GetTreeElementSegment(current_element).pose(q_in(GetTreeElementQNr(current_element)));
    else
      current_frame = GetTreeElementSegment(current_element).pose(0);

    Eigen::Isometry3d local_frame, global_frame;
    KDLToEigen(current_frame, local_frame);
    global_frame = parent_frame * local_frame;
    state.link_transforms[current_element.segment.getName()] = global_frame;
    if (current_element.segment.getName() != root_name_)
      state.joint_transforms[current_element.segment.getJoint().getName()] = global_frame;

    for (auto& child : current_element.children)
    {
      calculateTransformsHelper(state, q_in, child, global_frame);
    }
  }
}

void StateSolver::calculateTransforms(SceneModelState& state,
                                      const KDL::JntArray& q_in,
                                      const KDL::SegmentMap::const_iterator& it,
                                      const Eigen::Isometry3d& parent_frame) const
{
  calculateTransformsHelper(state, q_in, it, parent_frame);
}

}  // namespace tesseract_model_simple
