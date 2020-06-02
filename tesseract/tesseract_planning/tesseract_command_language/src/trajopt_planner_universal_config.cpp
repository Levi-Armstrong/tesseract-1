/**
 * @file trajopt_planner_default_config.cpp
 * @brief A TrajOpt planner configuration class with default values suitable for most applications
 *
 * @author Michael Ripperger
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <tesseract_command_language/trajopt_planner_universal_config.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/component_info_impl.h>

#include <tesseract_motion_planners/trajopt/config/utils.h>
#include <tesseract_motion_planners/core/utils.h>

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_planning
{

class InstructionResult
{
public:
  using Ptr = std::shared_ptr<InstructionResult>;
  using ConstPtr = std::shared_ptr<InstructionResult>;

  InstructionResult(const Instruction& instruction) : instruction(instruction) {}

  std::array<int, 2> trajectory_indexs;
  const Instruction& instruction;
};


/**
 * @brief If an intruction is not a move instruction or a known move instruction it will be nullptr
 *
 * This should be a one to one match withthe provided instructions
 */
using InstructionResults = std::vector<InstructionResult::Ptr>;

TrajOptPlannerUniversalConfig::TrajOptPlannerUniversalConfig(tesseract::Tesseract::ConstPtr tesseract_,
                                                             std::string manipulator_,
                                                             std::string link_,
                                                             tesseract_common::VectorIsometry3d tcp_)
  : tesseract(std::move(tesseract_)), manipulator(std::move(manipulator_)), link(std::move(link_)), tcp(std::move(tcp_))
{
}

TrajOptPlannerUniversalConfig::TrajOptPlannerUniversalConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                                                         const std::string& manipulator_,
                                                         const std::string& link_,
                                                         const Eigen::Isometry3d& tcp_)
  : TrajOptPlannerUniversalConfig(tesseract_, manipulator_, link_, tesseract_common::VectorIsometry3d(1, tcp_))
{
}

std::shared_ptr<trajopt::ProblemConstructionInfo> TrajOptPlannerUniversalConfig::generatePCI() const
{
  if (!checkUserInput())
    return nullptr;

  // -------- Construct the problem ------------
  // -------------------------------------------
  trajopt::ProblemConstructionInfo pci(tesseract);


//  if (!addInitTrajectory(pci))
//    return nullptr;

  std::vector<int> fixed_steps;
  addInstructions(pci, fixed_steps);

//  if (collision_constraint_config.enabled)
//    addCollisionConstraint(pci, fixed_steps);

//  if (collision_cost_config.enabled)
//    addCollisionCost(pci, fixed_steps);

//  if (smooth_velocities)
//    addVelocitySmoothing(pci, fixed_steps);

//  if (smooth_accelerations)
//    addAccelerationSmoothing(pci, fixed_steps);

//  if (smooth_jerks)
//    addJerkSmoothing(pci, fixed_steps);

//  if (configuration != nullptr)
//    addKinematicConfiguration(pci, fixed_steps);

//  if (!constraint_error_functions.empty())
//    addConstraintErrorFunctions(pci, fixed_steps);

//  if (avoid_singularity)
//    addAvoidSingularity(pci, fixed_steps);

  return std::make_shared<trajopt::ProblemConstructionInfo>(pci);
}

bool TrajOptPlannerUniversalConfig::generate()
{
  std::shared_ptr<trajopt::ProblemConstructionInfo> pci = generatePCI();
  if (!pci)
  {
    CONSOLE_BRIDGE_logError("Failed to construct problem from problem construction information");
    return false;
  }
  prob = trajopt::ConstructProblem(*pci);

  return true;
}

bool TrajOptPlannerUniversalConfig::checkUserInput() const
{
  // Check that parameters are valid
  if (tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In trajopt_array_planner: tesseract_ is a required parameter and has not been set");
    return false;
  }

  if (instructions.empty())
  {
    CONSOLE_BRIDGE_logError("TrajOpt Planner Config requires at least 2 waypoints");
    return false;
  }

//  if (tcp.size() != target_waypoints.size() && tcp.size() != 1)
//  {
//    std::stringstream ss;
//    ss << "Number of TCP transforms (" << tcp.size() << ") does not match the number of waypoints ("
//       << target_waypoints.size() << ") and is also not 1";
//    CONSOLE_BRIDGE_logError(ss.str().c_str());
//    return false;
//  }

  return true;
}

bool TrajOptPlannerUniversalConfig::addBasicInfo(trajopt::ProblemConstructionInfo& pci) const
{

  return true;
}

bool TrajOptPlannerUniversalConfig::addInitTrajectory(trajopt::ProblemConstructionInfo& pci) const
{
//  // Populate Init Info
//  pci.init_info.type = init_type;
//  if (init_type == trajopt::InitInfo::GIVEN_TRAJ)
//  {
//    pci.init_info.data = seed_trajectory;

//    // Add check to make sure if starts with joint waypoint that the seed trajectory also starts at this waypoint
//    // If it does not start this causes issues in trajopt and it will never converge.
//    if (isJointWaypointType(target_waypoints.front()->getType()))
//    {
//      const auto jwp = std::static_pointer_cast<JointWaypoint>(target_waypoints.front());
//      const Eigen::VectorXd position = jwp->getPositions(pci.kin->getJointNames());
//      for (int i = 0; i < static_cast<int>(pci.kin->numJoints()); ++i)
//      {
//        if (std::abs(position[i] - seed_trajectory(0, i)) > static_cast<double>(std::numeric_limits<float>::epsilon()))
//        {
//          std::stringstream ss;
//          ss << "Seed trajectory start position does not match starting joint waypoint position!";
//          ss << "    waypoint: " << position.transpose().matrix() << std::endl;
//          ss << "  seed start: " << seed_trajectory.row(0).transpose().matrix() << std::endl;
//          CONSOLE_BRIDGE_logError(ss.str().c_str());
//          return false;
//        }
//      }
//    }
//  }
//  if (init_type == trajopt::InitInfo::JOINT_INTERPOLATED)
//  {
//    if (seed_trajectory.size() != pci.kin->numJoints())
//    {
//      CONSOLE_BRIDGE_logError("Init type is set to JOINT_INTERPOLATED but seed_trajectory.size() != "
//                              "pci.kin->numJoints().");
//      return false;
//    }
//    pci.init_info.data = seed_trajectory;
//  }

//  return true;
}

trajopt::TermInfo::Ptr createNearJointStateTermInfo(const tesseract_planning::NearJointStateComponent& component,
                                                    const std::vector<std::string>& joint_names,
                                                    int index,
                                                    trajopt::TermType type)
{
  assert(static_cast<std::size_t>(component.target.size()) == joint_names.size());
  assert(component.joint_names.size() == joint_names.size());

  std::shared_ptr<trajopt::JointPosTermInfo> jp = std::make_shared<trajopt::JointPosTermInfo>();
  assert(std::equal(joint_names.begin(), joint_names.end(), component.joint_names.begin()));
  if (static_cast<std::size_t>(component.coeff.size()) == 1)
    jp->coeffs = std::vector<double>(joint_names.size(), component.coeff(0));  // Default value
  else if (static_cast<std::size_t>(component.coeff.size()) == joint_names.size())
    jp->coeffs = std::vector<double>(component.coeff.data(), component.coeff.data() + component.coeff.rows() * component.coeff.cols());

  jp->targets = std::vector<double>(component.target.data(), component.target.data() + component.target.size());
  jp->first_step = index;
  jp->last_step = index;
  jp->name = component.getName() + "(" + std::to_string(index) + ")";
  jp->term_type = trajopt::TT_COST;

  return jp;
}

trajopt::TermInfo::Ptr createCartesianWaypoint(const CartesianWaypoint& c_wp,
                                               int index,
                                               std::string working_frame,
                                               Eigen::Isometry3d tcp,
                                               const Eigen::VectorXd& coeffs,
                                               std::string link,
                                               trajopt::TermType type)
{
  auto pose_info = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_info->term_type = type;
  pose_info->name = "cartesian_waypoint_" + std::to_string(index);

  pose_info->link = link;
  pose_info->tcp = tcp;

  pose_info->timestep = index;
  pose_info->xyz = c_wp.translation();
  Eigen::Quaterniond q(c_wp.linear());
  pose_info->wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  pose_info->target = working_frame;

  if (coeffs.size() == 1)
  {
    pose_info->pos_coeffs = Eigen::Vector3d::Constant(coeffs(0));
    pose_info->rot_coeffs = Eigen::Vector3d::Constant(coeffs(0));
  }
  else if (coeffs.size() == 6)
  {
    pose_info->pos_coeffs = coeffs.head<3>();
    pose_info->rot_coeffs = coeffs.tail<3>();
  }

  return pose_info;
}

void createCartesianComponents(trajopt::ProblemConstructionInfo &pci,
                               const CartesianWaypoint& c_wp,
                               int index,
                               std::string working_frame,
                               Eigen::Isometry3d tcp,
                               const std::vector<ComponentInfo>& components,
                               std::string link,
                               trajopt::TermType type)
{
  trajopt::TermInfo::Ptr info;
  for (const auto& component : components)
  {
    switch (component.getType())
    {
      case static_cast<int>(ComponentTypes::FIXED):
      {
        const auto* local = component.cast_const<FixedComponent>();
        info = createCartesianWaypoint(c_wp, index, working_frame, tcp, local->coeff, link, type);
        break;
      }
      case static_cast<int>(ComponentTypes::NEAR_JOINT_STATE):
      {
        const auto* local = component.cast_const<NearJointStateComponent>();
        info = createNearJointStateTermInfo(*local, pci.kin->getJointNames(), index, type);
        break;
      }
      default:
      {
        throw std::runtime_error("Invalid or unsupport component type for cartesian waypoint!");
      }
    }

    if (type == trajopt::TT_CNT)
      pci.cnt_infos.push_back(info);
    else if (type == trajopt::TT_COST)
      pci.cost_infos.push_back(info);
    else
      throw std::runtime_error("Invalid trajopt term type!");
  }
}

trajopt::TermInfo::Ptr createJointWaypoint(const JointWaypoint& j_wp,
                                           int index,
                                           Eigen::Isometry3d tcp,
                                           const Eigen::VectorXd& coeffs,
                                           std::string link,
                                           trajopt::TermType type)
{
  auto joint_info = std::make_shared<trajopt::JointPosTermInfo>();
  if (coeffs.size() == 1)
    joint_info->coeffs = std::vector<double>(static_cast<std::size_t>(j_wp.size()), coeffs(0));
  else if (coeffs.size() == j_wp.size())
    joint_info->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());

  joint_info->targets = std::vector<double>(j_wp.data(), j_wp.data() + j_wp.rows() * j_wp.cols());
  joint_info->first_step = index;
  joint_info->last_step = index;
  joint_info->name = "joint_waypoint_" + std::to_string(index);
  joint_info->term_type = type;

  return joint_info;
}

void createJointComponents(trajopt::ProblemConstructionInfo &pci,
                           const JointWaypoint& j_wp,
                           int index,
                           const std::vector<ComponentInfo>& components,
                           std::string link,
                           trajopt::TermType type)
{
  trajopt::TermInfo::Ptr info;
  for (const auto& component : components)
  {
    switch (component.getType())
    {
      case static_cast<int>(ComponentTypes::FIXED):
      {
        const auto* local = component.cast_const<FixedComponent>();
        info = createJointWaypoint(j_wp, index, Eigen::Isometry3d::Identity(), local->coeff, link, type);
        break;
      }
      case static_cast<int>(ComponentTypes::NEAR_JOINT_STATE):
      {
        const auto* local = component.cast_const<NearJointStateComponent>();
        info = createNearJointStateTermInfo(*local, pci.kin->getJointNames(), index, type);
        break;
      }
      default:
      {
        throw std::runtime_error("Invalid or unsupport component type for cartesian waypoint!");
      }
    }

    if (type == trajopt::TT_CNT)
      pci.cnt_infos.push_back(info);
    else if (type == trajopt::TT_COST)
      pci.cost_infos.push_back(info);
    else
      throw std::runtime_error("Invalid trajopt term type!");
  }
}

void createCompositeComponents(trajopt::ProblemConstructionInfo &pci,
                               const std::vector<ComponentInfo>& components,
                               trajopt::TermType type)
{
  trajopt::TermInfo::Ptr info;
  for (const auto& component : components)
  {
    switch (component.getType())
    {
      case static_cast<int>(ComponentTypes::VELOCITY_SMOOTHING):
      {
        const auto* vs = component.cast_const<VelocitySmoothingComponent>();
        if (vs->coeff.size() == 0)
          info = tesseract_motion_planners::createSmoothVelocityTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints()));
        else if (vs->coeff.size() == 1)
          info = tesseract_motion_planners::createSmoothVelocityTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints()), vs->coeff(0));
        else
          info = tesseract_motion_planners::createSmoothVelocityTermInfo(pci.basic_info.n_steps, vs->coeff);
        break;
      }
      case static_cast<int>(ComponentTypes::ACCELERATION_SMOOTHING):
      {
        const auto* vs = component.cast_const<AccelerationSmoothingComponent>();
        if (vs->coeff.size() == 0)
          info = tesseract_motion_planners::createSmoothAccelerationTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints()));
        else if (vs->coeff.size() == 1)
          info = tesseract_motion_planners::createSmoothAccelerationTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints()), vs->coeff(0));
        else
          info = tesseract_motion_planners::createSmoothAccelerationTermInfo(pci.basic_info.n_steps, vs->coeff);

        break;
      }
      case static_cast<int>(ComponentTypes::JERK_SMOOTHING):
      {
        const auto* vs = component.cast_const<AccelerationSmoothingComponent>();
        if (vs->coeff.size() == 0)
          info = tesseract_motion_planners::createSmoothJerkTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints()));
        else if (vs->coeff.size() == 1)
          info = tesseract_motion_planners::createSmoothJerkTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints()), vs->coeff(0));
        else
          info = tesseract_motion_planners::createSmoothJerkTermInfo(pci.basic_info.n_steps, vs->coeff);

        break;
      }
      default:
      {
        throw std::runtime_error("Invalid or unsupport component type for cartesian waypoint!");
      }
    }

    info->term_type = type;
    if (type == trajopt::TT_CNT)
      pci.cnt_infos.push_back(info);
    else if (type == trajopt::TT_COST)
      pci.cost_infos.push_back(info);
    else
      throw std::runtime_error("Invalid trajopt term type!");
  }
}


void TrajOptPlannerUniversalConfig::addInstructions(trajopt::ProblemConstructionInfo &pci,
                                                    std::vector<int> &fixed_steps) const
{
  // Assign Kinematics object
  pci.kin = pci.getManipulator(manipulator);

  if (pci.kin == nullptr)
  {
    std::string error_msg = "In trajopt_array_planner: manipulator_ does not exist in kin_map_";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Check and make sure it does not contain any composite instruction
  const PlanInstruction* start_instruction {nullptr};
  for (const auto& instruction : instructions)
  {
    if (instruction.isComposite())
      throw std::runtime_error("Trajopt planner does not support child composite instructions.");

    if (start_instruction == nullptr && instruction.isPlan())
      start_instruction = instruction.cast_const<PlanInstruction>();
  }

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = tesseract->getEnvironmentConst();
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), pci.kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& adjacency_links = map.getActiveLinkNames();

  // Transform move instructions into trajopt cost and constraints
  const PlanInstruction* prev_plan_instruction {nullptr};
  int index = 0;
  for (std::size_t i = 0; i < instructions.size(); ++i)
  {
    const auto& instruction = instructions[i];
    const CompositeInstruction* seed_info = seed[i].cast_const<CompositeInstruction>();
    if (instruction.isPlan())
    {
      assert(instruction.getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      const Waypoint& wp = plan_instruction->getWaypoint();
      const std::string& working_frame = plan_instruction->getWorkingFrame();
      const Eigen::Isometry3d& tcp = plan_instruction->getTCP();

      if (plan_instruction->isLinear())
      {
        tesseract_planning::CompositeInstruction composite;

        /** @todo This should also handle if waypoint type is joint */
        const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
        if (prev_plan_instruction)
        {
          /** @todo This should also handle if waypoint type is joint */
          const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();

          tesseract_common::VectorIsometry3d poses = tesseract_motion_planners::interpolate(*pre_cwp, *cur_cwp, static_cast<int>(seed_info->size()));
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            createCartesianComponents(pci, poses[p], index, working_frame, tcp, plan_instruction->getPathCosts(), link, trajopt::TT_COST);
            createCartesianComponents(pci, poses[p], index, working_frame, tcp, plan_instruction->getPathConstraints(), link, trajopt::TT_CNT);


            /** @todo Add createDynamicCartesianWaypointTermInfo */
            /* Check if this cartesian waypoint is dynamic
             * (i.e. defined relative to a frame that will move with the kinematic chain)
             */
            auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
            if (it != adjacency_links.end())
              throw std::runtime_error("Dynamic cartesian waypoint is currently not supported!");

            ++index;
          }
        }

        // Add final point with waypoint costs and contraints
        createCartesianComponents(pci, *cur_cwp, index, working_frame, tcp, plan_instruction->getCosts(), link, trajopt::TT_COST);
        createCartesianComponents(pci, *cur_cwp, index, working_frame, tcp, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

        /** @todo Add createDynamicCartesianWaypointTermInfo */
        /* Check if this cartesian waypoint is dynamic
         * (i.e. defined relative to a frame that will move with the kinematic chain)
         */
        auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
        if (it != adjacency_links.end())
          throw std::runtime_error("Dynamic cartesian waypoint is currently not supported!");

        ++index;
        prev_plan_instruction = plan_instruction;
      }
      else if (plan_instruction->isFreespace())
      {
        tesseract_planning::CompositeInstruction composite;

        /** @todo This should also handle if waypoint type is cartesian */
        const auto* cur_cwp = plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
        if (prev_plan_instruction)
        {
          /** @todo This should also handle if waypoint type is cartesian */
          const auto* pre_cwp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();

          Eigen::MatrixXd states = tesseract_motion_planners::interpolate(*pre_cwp, *cur_cwp, static_cast<int>(seed_info->size()));
          // Add intermediate points with path costs and constraints
          for (long i = 1; i < states.cols() - 1; ++i)
          {
            createJointComponents(pci, states.col(i), index, plan_instruction->getPathCosts(), link, trajopt::TT_COST);
            createJointComponents(pci, states.col(i), index, plan_instruction->getPathConstraints(), link, trajopt::TT_CNT);

            ++index;
          }
        }

        // Add final point with waypoint costs and contraints
        createJointComponents(pci, *cur_cwp, index, plan_instruction->getCosts(), link, trajopt::TT_COST);
        createJointComponents(pci, *cur_cwp, index, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

        ++index;
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      prev_plan_instruction = plan_instruction;
    }
  }

  // Setup Basic Info
  pci.basic_info.n_steps = index - 1;
  pci.basic_info.manip = manipulator;
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;
  pci.basic_info.convex_solver = optimizer;

  createCompositeComponents(pci, instructions.getCosts(), trajopt::TT_COST);
  createCompositeComponents(pci, instructions.getConstraints(), trajopt::TT_CNT);
}

void TrajOptPlannerUniversalConfig::addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                                                   const std::vector<int>& fixed_steps) const
{
//  // Calculate longest valid segment length
//  const Eigen::MatrixX2d& limits = pci.kin->getLimits();
//  double length = 0;
//  double extent = (limits.col(1) - limits.col(0)).norm();
//  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
//  {
//    length = std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);
//  }
//  else if (longest_valid_segment_fraction > 0)
//  {
//    length = longest_valid_segment_fraction * extent;
//  }
//  else if (longest_valid_segment_length > 0)
//  {
//    length = longest_valid_segment_length;
//  }
//  else
//  {
//    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
//  }

//  // Create a default collision term info
//  trajopt::TermInfo::Ptr ti = createCollisionTermInfo(pci.basic_info.n_steps,
//                                                      collision_cost_config.buffer_margin,
//                                                      collision_constraint_config.safety_margin_buffer,
//                                                      collision_cost_config.type,
//                                                      collision_cost_config.use_weighted_sum,
//                                                      collision_cost_config.coeff,
//                                                      contact_test_type,
//                                                      length,
//                                                      "collision_cost",
//                                                      false);

//  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
//  std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
//  if (special_collision_cost)
//  {
//    for (auto& info : ct->info)
//    {
//      info = special_collision_cost;
//    }
//  }
//  ct->fixed_steps = fixed_steps;

//  pci.cost_infos.push_back(ct);
}

void TrajOptPlannerUniversalConfig::addCollisionConstraint(trajopt::ProblemConstructionInfo& pci,
                                                         const std::vector<int>& fixed_steps) const
{
//  // Calculate longest valid segment length
//  const Eigen::MatrixX2d& limits = pci.kin->getLimits();
//  double length = 0;
//  double extent = (limits.col(1) - limits.col(0)).norm();
//  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
//  {
//    length = std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);
//  }
//  else if (longest_valid_segment_fraction > 0)
//  {
//    length = longest_valid_segment_fraction * extent;
//  }
//  else if (longest_valid_segment_length > 0)
//  {
//    length = longest_valid_segment_length;
//  }
//  else
//  {
//    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
//  }

//  // Create a default collision term info
//  trajopt::TermInfo::Ptr ti = createCollisionTermInfo(pci.basic_info.n_steps,
//                                                      collision_constraint_config.safety_margin,
//                                                      collision_constraint_config.safety_margin_buffer,
//                                                      collision_constraint_config.type,
//                                                      collision_cost_config.use_weighted_sum,
//                                                      collision_constraint_config.coeff,
//                                                      contact_test_type,
//                                                      length,
//                                                      "collision_cnt",
//                                                      true);

//  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
//  std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
//  if (special_collision_constraint)
//  {
//    for (auto& info : ct->info)
//    {
//      info = special_collision_constraint;
//    }
//  }
//  ct->fixed_steps = fixed_steps;

//  pci.cnt_infos.push_back(ct);
}

void TrajOptPlannerUniversalConfig::addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci,
                                                              const std::vector<int>& fixed_steps) const
{
//  for (std::size_t i = 0; i < constraint_error_functions.size(); ++i)
//  {
//    auto& c = constraint_error_functions[i];
//    trajopt::TermInfo::Ptr ti = createUserDefinedTermInfo(
//        pci.basic_info.n_steps, std::get<0>(c), std::get<1>(c), "user_defined_" + std::to_string(i));

//    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
//    std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
//    ef->term_type = trajopt::TT_CNT;
//    ef->constraint_type = std::get<2>(c);
//    ef->coeff = std::get<3>(c);
//    ef->first_step = 0;
//    ef->last_step = pci.basic_info.n_steps - 1;
//    ef->fixed_steps = fixed_steps;

//    pci.cnt_infos.push_back(ef);
//  }
}

void TrajOptPlannerUniversalConfig::addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                                                      const std::vector<int>& /*fixed_steps*/) const
{
//  trajopt::TermInfo::Ptr ti = createAvoidSingularityTermInfo(pci.basic_info.n_steps, link, avoid_singularity_coeff);
//  pci.cost_infos.push_back(ti);
}

}  // namespace tesseract_motion_planners
