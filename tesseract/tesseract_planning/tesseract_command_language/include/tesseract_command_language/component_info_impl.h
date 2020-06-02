#ifndef TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
#define TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H

#include <Eigen/Geometry>
#include <vector>

namespace tesseract_planning
{

enum class ComponentTypes : int
{
  FIXED,
  CARTESIAN_X_TOL,
  CARTESIAN_Y_TOL,
  CARTESIAN_Z_TOL,
  CARTESIAN_XY_TOL,
  CARTESIAN_XZ_TOL,
  CARTESIAN_YZ_TOL,
  CARTESIAN_XYZ_TOL,
  JOINT_TOL,
  VELOCITY_TOL,
  AVOID_SINGULARITY,
  NEAR_JOINT_STATE,
  VELOCITY_SMOOTHING,
  ACCELERATION_SMOOTHING,
  JERK_SMOOTHING,
};

class FixedComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::FIXED); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return false; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Fixed Component"};
};

class CartesianXTolComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::CARTESIAN_X_TOL); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  double target{0};
  double min{0};
  double max{0};

  /** @brief This is the coefficient/weight that may be used by the planner */
  double coeff {1};

  /** @brief The name of the component */
  std::string name {"Cartesian X Tolerance Component"};
};

class VelocityComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_TOL); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  double target{0};
  double min{0};
  double max{0};

  /** @brief This is the coefficient/weight that may be used by the planner */
  double coeff {1};

  /** @brief The name of the component */
  std::string name {"Velocity Component"};
};

class VelocitySmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_SMOOTHING); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Velocity Smoothing Component"};
};

class AccelerationSmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::ACCELERATION_SMOOTHING); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Acceleration Smoothing Component"};
};

class JerkSmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::JERK_SMOOTHING); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Jerk Smoothing Component"};
};

/** @brief This component tells the planner to avoid singularity for a given move instruction */
class AvoidSingularityComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::AVOID_SINGULARITY); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Avoid Singularity Component"};
};

/** @brief This component tells the planner to stay near a joint target for a given move instruction  */
class NearJointStateComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::NEAR_JOINT_STATE); }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief The joint state target to stay near for a given move instruction */
  Eigen::VectorXd target;

  /** @brief The joint names for the provided target */
  std::vector<std::string> joint_names;

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Near Joint State Component"};
};
}
#endif // TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
