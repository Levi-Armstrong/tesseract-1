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

  if (!addBasicInfo(pci))
    return nullptr;

  if (!addInitTrajectory(pci))
    return nullptr;

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
  pci.kin = pci.getManipulator(manipulator);

  if (pci.kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("In trajopt_array_planner: manipulator_ does not exist in kin_map_");
    return false;
  }

  // Populate Basic Info
//  pci.basic_info.n_steps = static_cast<int>(target_waypoints.size());
//  pci.basic_info.manip = manipulator;
//  pci.basic_info.start_fixed = false;
//  pci.basic_info.use_time = false;
//  pci.basic_info.convex_solver = optimizer;

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

trajopt::TermInfo::Ptr createCartesianWaypoint(const CartesianWaypoint* c_wp,
                                               int index,
                                               std::string working_frame,
                                               Eigen::Isometry3d tcp,
                                               std::string link,
                                               trajopt::TermType type)
{
  auto pose_info = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_info->term_type = type;
  pose_info->name = "cartesian_waypoint_" + std::to_string(index);

  pose_info->link = link;
  pose_info->tcp = tcp;

  pose_info->timestep = index;
  pose_info->xyz = c_wp->translation();
  Eigen::Quaterniond q(c_wp->linear());
  pose_info->wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  pose_info->target = working_frame;

//              const Eigen::VectorXd& coeffs = waypoint->getCoefficients();
  Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(6);
  assert(coeffs.size() == 6);
  pose_info->pos_coeffs = coeffs.head<3>();
  pose_info->rot_coeffs = coeffs.tail<3>();

  return pose_info;
}

void createCartesianComponents(tesseract_motion_planners::WaypointTermInfo& term_info,
                               const CartesianWaypoint* c_wp,
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
        info = createCartesianWaypoint(c_wp, index, working_frame, tcp, link, trajopt::TT_COST);
        break;
      }
      case static_cast<int>(ComponentTypes::CARTESIAN_X_TOL):
      {
        info = createCartesianWaypoint(c_wp, index, working_frame, tcp, link, trajopt::TT_COST);
        break;
      }
      default:
      {
        throw std::runtime_error("Invalid or unsupport component type for cartesian waypoint!");
      }
    }

    if (type == trajopt::TT_CNT)
      term_info.cnt.push_back(info);
    else if (type == trajopt::TT_COST)
      term_info.cost.push_back(info);
    else
      throw std::runtime_error("Invalid trajopt term type!");
  }
}

trajopt::TermInfo::Ptr createJointWaypoint(const JointWaypoint* j_wp,
                                           int index,
                                           Eigen::Isometry3d tcp,
                                           std::string link,
                                           trajopt::TermType type)
{
  auto joint_info = std::make_shared<trajopt::JointPosTermInfo>();
  Eigen::VectorXd coeffs = Eigen::VectorXd::Ones(j_wp->size());
  if (coeffs.size() != j_wp->size())
    joint_info->coeffs = std::vector<double>(static_cast<std::size_t>(j_wp->size()), coeffs(0));  // Default value
  else
    joint_info->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());
  joint_info->targets = std::vector<double>(j_wp->data(), j_wp->data() + j_wp->rows() * j_wp->cols());
  joint_info->first_step = index;
  joint_info->last_step = index;
  joint_info->name = "joint_waypoint_" + std::to_string(index);
  joint_info->term_type = type;

  return joint_info;
}



void createJointComponents(tesseract_motion_planners::WaypointTermInfo& term_info,
                           const JointWaypoint* c_wp,
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
        info = createJointWaypoint(c_wp, index, Eigen::Isometry3d::Identity(), link, trajopt::TT_COST);
        break;
      }
      case static_cast<int>(ComponentTypes::CARTESIAN_X_TOL):
      {
        info = createJointWaypoint(c_wp, index, Eigen::Isometry3d::Identity(), link, trajopt::TT_COST);
        break;
      }
      default:
      {
        throw std::runtime_error("Invalid or unsupport component type for cartesian waypoint!");
      }
    }

    if (type == trajopt::TT_CNT)
      term_info.cnt.push_back(info);
    else if (type == trajopt::TT_COST)
      term_info.cost.push_back(info);
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
  for (const auto& instruction : instructions)
  {
    if (instruction.isPlan())
    {
      if (instruction.getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION))
      {
        const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
        const Waypoint& wp = plan_instruction->getWaypoint();
        const std::string& working_frame = plan_instruction->getWorkingFrame();
        const Eigen::Isometry3d& tcp = plan_instruction->getTCP();

        if (wp.getType() == static_cast<int>(WaypointType::CARTESIAN_WAYPOINT))
        {
          const CartesianWaypoint* c_wp = wp.cast_const<CartesianWaypoint>();

          // If nullptr this this is the start position
          if (prev_plan_instruction == nullptr)
          {
            tesseract_motion_planners::WaypointTermInfo term_info;
            createCartesianComponents(term_info, c_wp, index, working_frame, tcp, plan_instruction->getCosts(), link, trajopt::TT_COST);
            createCartesianComponents(term_info, c_wp, index, working_frame, tcp, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

            /** @todo Add createDynamicCartesianWaypointTermInfo */
            /* Check if this cartesian waypoint is dynamic
             * (i.e. defined relative to a frame that will move with the kinematic chain)
             */
            auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
            if (it != adjacency_links.end())
              throw std::runtime_error("Dynamic cartesian waypoint is currently not supported!");

            prev_plan_instruction = plan_instruction;
          }
          else
          {
            if (plan_instruction->isLinear())
            {
              /** @todo Interpolate cartesian */
              /** @todo Interpolate seed trajectory if exist */
            }
            else if (plan_instruction->isFreespace())
            {
              /** @todo Interpolate seed trajectory if exist, if not use stationary state. */
            }
            else
            {
              throw std::runtime_error("Circular Move Instruction Type is not supported!");
            }

            // Since we are currently not interpolating we will just add the provided waypoint
            tesseract_motion_planners::WaypointTermInfo term_info;
            createCartesianComponents(term_info, c_wp, index, working_frame, tcp, plan_instruction->getCosts(), link, trajopt::TT_COST);
            createCartesianComponents(term_info, c_wp, index, working_frame, tcp, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

            /** @todo Add createDynamicCartesianWaypointTermInfo */
            /* Check if this cartesian waypoint is dynamic
             * (i.e. defined relative to a frame that will move with the kinematic chain)
             */
            auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
            if (it != adjacency_links.end())
              throw std::runtime_error("Dynamic cartesian waypoint is currently not supported!");
          }
        }
        else if (wp.getType() == static_cast<int>(WaypointType::JOINT_WAYPOINT))
        {
          const JointWaypoint* j_wp = wp.cast_const<JointWaypoint>();

          // If nullptr this this is the start position
          if (prev_plan_instruction == nullptr)
          {
            tesseract_motion_planners::WaypointTermInfo term_info;
            createJointComponents(term_info, j_wp, index, plan_instruction->getCosts(), link, trajopt::TT_COST);
            createJointComponents(term_info, j_wp, index, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

            prev_plan_instruction = plan_instruction;
          }
          else
          {
            if (plan_instruction->isLinear())
            {
              /** @todo Interpolate cartesian */
              /** @todo Interpolate seed trajectory if exist */
            }
            else if (plan_instruction->isFreespace())
            {
              /** @todo Interpolate seed trajectory if exist, if not use stationary state. */
            }
            else
            {
              throw std::runtime_error("Circular Move Instruction Type is not supported!");
            }

            // Since we are currently not interpolating we will just add the provided waypoint
            tesseract_motion_planners::WaypointTermInfo term_info;
            createJointComponents(term_info, j_wp, index, plan_instruction->getCosts(), link, trajopt::TT_COST);
            createJointComponents(term_info, j_wp, index, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

            /* Update the first and last step for the costs
             * Certain costs (collision checking and configuration) should not be applied to start and end states
             * that are incapable of changing (i.e. joint positions). Therefore, the first and last indices of these
             * costs (which equal 0 and num_steps-1 by default) should be changed to exclude those states
             */
            if (!plan_instruction->getConstraints().empty())
              fixed_steps.push_back(index);
          }
        }
        else
        {
          throw std::runtime_error("Trajopt planner does not support waypoint type " + std::to_string(wp.getType()));
        }
        prev_plan_instruction = plan_instruction;
      }
      ++index;
    }
  }

  pci.basic_info.n_steps = index - 1;

  createCompositeComponents(pci, instructions.getCosts(), trajopt::TT_COST);
  createCompositeComponents(pci, instructions.getCosts(), trajopt::TT_COST);
}

void TrajOptPlannerUniversalConfig::addKinematicConfiguration(trajopt::ProblemConstructionInfo& pci,
                                                              const std::vector<int>& fixed_steps) const
{
//  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
//  if (fixed_steps.empty())
//  {
//    trajopt::TermInfo::Ptr ti =
//        createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//    std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
//    pci.cost_infos.push_back(jp);
//  }
//  else if (fixed_steps.size() == 1)
//  {
//    if (fixed_steps[0] == 0)
//    {
//      trajopt::TermInfo::Ptr ti =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
//      ++(jp->first_step);
//      pci.cost_infos.push_back(jp);
//    }
//    else if (fixed_steps[0] == (pci.basic_info.n_steps - 1))
//    {
//      trajopt::TermInfo::Ptr ti =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
//      --(jp->last_step);
//      pci.cost_infos.push_back(jp);
//    }
//    else
//    {
//      trajopt::TermInfo::Ptr ti1 =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp1 = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti1);
//      jp1->last_step = fixed_steps[0] - 1;
//      pci.cost_infos.push_back(jp1);

//      trajopt::TermInfo::Ptr ti2 =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp2 = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti2);
//      jp2->first_step = fixed_steps[0] + 1;
//      pci.cost_infos.push_back(jp2);
//    }
//  }
//  else
//  {
//    if (fixed_steps.front() != 0)
//    {
//      trajopt::TermInfo::Ptr ti =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
//      jp->last_step = fixed_steps.front() - 1;
//      pci.cost_infos.push_back(jp);
//    }

//    for (size_t i = 1; i < fixed_steps.size(); ++i)
//    {
//      trajopt::TermInfo::Ptr ti =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
//      jp->first_step = fixed_steps[i - 1] + 1;
//      jp->last_step = fixed_steps[i] - 1;
//      pci.cost_infos.push_back(jp);
//    }

//    if (fixed_steps.back() != (pci.basic_info.n_steps - 1))
//    {
//      trajopt::TermInfo::Ptr ti =
//          createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);
//      std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
//      jp->first_step = fixed_steps.back() + 1;
//      pci.cost_infos.push_back(jp);
//    }
//  }
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
