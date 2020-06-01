#ifndef TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
#define TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H

#include <Eigen/Geometry>

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
  VELOCITY_SMOOTHING,
  ACCELERATION_SMOOTHING,
  JERK_SMOOTHING,
};

class FixedComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::FIXED); }
  bool isCompositeInstructionSupported() const { return false; }
};

class CartesianXTolComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::CARTESIAN_X_TOL); }
  bool isCompositeInstructionSupported() const { return true; }

  double target;
  double min;
  double max;
  double coeff;
};

class VelocityComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_TOL); }
  bool isCompositeInstructionSupported() const { return true; }

  double target;
  double min;
  double max;
  double coeff;
};

class VelocitySmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_SMOOTHING); }
  bool isCompositeInstructionSupported() const { return true; }

  Eigen::VectorXd coeff;
};

class AccelerationSmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::ACCELERATION_SMOOTHING); }
  bool isCompositeInstructionSupported() const { return true; }

  Eigen::VectorXd coeff;
};

class JerkSmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::JERK_SMOOTHING); }
  bool isCompositeInstructionSupported() const { return true; }

  Eigen::VectorXd coeff;
};

class AvoidSingularityComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::AVOID_SINGULARITY); }
  bool isCompositeInstructionSupported() const { return true; }

  Eigen::VectorXd coeff;
};
}
#endif // TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
