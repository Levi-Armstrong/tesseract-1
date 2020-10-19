#ifndef TESSERACT_SCENE_GRAPH_SRDF_TYPES_H
#define TESSERACT_SCENE_GRAPH_SRDF_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/visibility_control.h>

#ifdef SWIG

%template(GroupOPWKinematics) std::unordered_map<std::string, tesseract_scene_graph::OPWKinematicParameters>;
%template(GroupROPKinematics) std::unordered_map<std::string, tesseract_scene_graph::ROPKinematicParameters>;
%template(GroupREPKinematics) std::unordered_map<std::string, tesseract_scene_graph::REPKinematicParameters>;

#endif // SWIG


namespace tesseract_scene_graph
{
/** @brief A structure to hold opw kinematics data */
struct TESSERACT_SCENE_GRAPH_PUBLIC OPWKinematicParameters
{
#ifndef SWIG
  double a1{ 0 }, a2{ 0 }, b{ 0 }, c1{ 0 }, c2{ 0 }, c3{ 0 }, c4{ 0 };
  double offsets[6]{ 0, 0, 0, 0, 0, 0 };
  signed char sign_corrections[6]{ 1, 1, 1, 1, 1, 1 };
#else // SWIG
  double a1, a2, b, c1, c2, c3, c4;
  double offsets[6];
  signed char sign_corrections[6];
#endif // SWIG
};

struct TESSERACT_SCENE_GRAPH_PUBLIC ROPKinematicParameters
{
  std::string manipulator_group;
  std::string manipulator_ik_solver;
  double manipulator_reach;
  std::string positioner_group;
  std::string positioner_fk_solver;
  std::unordered_map<std::string, double> positioner_sample_resolution;
};

struct TESSERACT_SCENE_GRAPH_PUBLIC REPKinematicParameters
{
  std::string manipulator_group;
  std::string manipulator_ik_solver;
  double manipulator_reach;
  std::string positioner_group;
  std::string positioner_fk_solver;
  std::unordered_map<std::string, double> positioner_sample_resolution;
};

using GroupsJointState = std::unordered_map<std::string, double>;
using GroupsJointStates = std::unordered_map<std::string, GroupsJointState>;
using GroupJointStates = std::unordered_map<std::string, GroupsJointStates>;
using GroupsTCPs = std::unordered_map<std::string, Eigen::Isometry3d>;
using GroupTCPs = std::unordered_map<std::string, GroupsTCPs>;
using ChainGroup = std::vector<std::pair<std::string, std::string>>;
using ChainGroups = std::unordered_map<std::string, ChainGroup>;
using JointGroup = std::vector<std::string>;
using JointGroups = std::unordered_map<std::string, JointGroup>;
using LinkGroup = std::vector<std::string>;
using LinkGroups = std::unordered_map<std::string, LinkGroup>;
using GroupNames = std::vector<std::string>;
using GroupROPKinematics = std::unordered_map<std::string, ROPKinematicParameters>;
using GroupREPKinematics = std::unordered_map<std::string, REPKinematicParameters>;
using GroupOPWKinematics = std::unordered_map<std::string, OPWKinematicParameters>;

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_SRDF_TYPES_H
