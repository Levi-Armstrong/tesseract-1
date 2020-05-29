#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H

#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_collision_config.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{

/**
 * @brief Default configuration to setup TrajOpt planner.
 *
 * While there are many parameters that can be set, the majority of them have defaults that will be suitable for many
 * problems. These are always required: tesseract_, maninpulator_, link_, tcp_
 *
 */
struct TrajOptPlannerUniversalConfig : public tesseract_motion_planners::TrajOptPlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TrajOptPlannerUniversalConfig>;
  using ConstPtr = std::shared_ptr<const TrajOptPlannerUniversalConfig>;

  TrajOptPlannerUniversalConfig(tesseract::Tesseract::ConstPtr tesseract_,
                                std::string manipulator_,
                                std::string link_,
                                tesseract_common::VectorIsometry3d tcp_);

  TrajOptPlannerUniversalConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                                const std::string& manipulator_,
                                const std::string& link_,
                                const Eigen::Isometry3d& tcp_);

  /** @brief Function for creating a ProblemConstructionInfo from the planner configuration */
  virtual std::shared_ptr<trajopt::ProblemConstructionInfo> generatePCI() const;

  /** @brief Generates the TrajOpt problem and saves the result internally */
  bool generate() override;

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;
  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator;
  /** @brief This is the tip link in the kinematics object used for the cartesian positions ***REQUIRED*** */
  std::string link;

  /** @brief The QP solver used in the SQP optimization routine */
  sco::ModelType optimizer = sco::ModelType::AUTO_SOLVER;

  /**
   * @brief Vector of TCP transforms. This should contain either one transform to be applied to all waypoints
   * or a separate transform for each waypoint
   */
  tesseract_common::VectorIsometry3d tcp;

  /**
   * @brief The program instruction
   * This must containt a minimum of two move instruction the first move instruction is the start state
   */
  tesseract_planning::CompositeInstruction instructions;

  /** @brief For a linear move instruction this is used to determine the number of states to be generated */
  double longest_cartesian_translation_segment {0.05};

  /** @brief For a linear move instruction this is used to determine the number of states to be generated */
  double longest_cartesian_rotation_segment {0.05};

  /** @brief For a joint move instruction this the number of states used */
  int joint_move_num_states {20};

  /** @brief Selects the type of initialization used for raster path. If GIVEN_TRAJ, then the seed_trajectory_ must be
   * set */
  trajopt::InitInfo::Type init_type = trajopt::InitInfo::STATIONARY;

  /** @brief The trajectory used as the optimization seed when init_type_ is set to GIVEN_TRAJ */
  trajopt::TrajArray seed_trajectory;

protected:
  bool checkUserInput() const;
  bool addBasicInfo(trajopt::ProblemConstructionInfo& pci) const;
  bool addInitTrajectory(trajopt::ProblemConstructionInfo& pci) const;
  void addInstructions(trajopt::ProblemConstructionInfo& pci, std::vector<int>& fixed_steps) const;
  void addKinematicConfiguration(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addCollisionCost(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addCollisionConstraint(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addVelocitySmoothing(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addAccelerationSmoothing(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addJerkSmoothing(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;
  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci, const std::vector<int>& fixed_steps) const;

  std::vector<std::pair<long, long>> instruction_to_trajectory_map_;
};

}

#endif // TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H
