#ifndef TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
#define TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H

namespace tesseract_planning
{

enum class ComponentTypes : int
{
  FIXED,
  LINEAR,
  FREESPACE,
  CARTESIAN_X_TOL,
  CARTESIAN_Y_TOL,
  CARTESIAN_Z_TOL,
  CARTESIAN_XY_TOL,
  CARTESIAN_XZ_TOL,
  CARTESIAN_YZ_TOL,
  CARTESIAN_XYZ_TOL,
  JOINT_TOL,
  VELOCITY_TOL,
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

class LinearComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::LINEAR); }
  bool isCompositeInstructionSupported() const { return false; }
};

class FreespaceComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::FREESPACE); }
  bool isCompositeInstructionSupported() const { return false; }
};

class CartesianXTolComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::CARTESIAN_X_TOL); }
  bool isCompositeInstructionSupported() const { return false; }

  double x_min;
  double x_max;
  double target;
};

class VelocityComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_TOL); }
  bool isCompositeInstructionSupported() const { return true; }

  double target;
  double min;
  double max;
};

class VelocitySmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_SMOOTHING); }
  bool isCompositeInstructionSupported() const { return true; }
};

class AccelerationSmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::ACCELERATION_SMOOTHING); }
  bool isCompositeInstructionSupported() const { return true; }
};

class JerkSmoothingComponent
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::JERK_SMOOTHING); }
  bool isCompositeInstructionSupported() const { return true; }
};
}
#endif // TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
