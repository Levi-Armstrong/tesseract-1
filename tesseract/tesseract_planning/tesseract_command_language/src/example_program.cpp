#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/component_info.h>
#include <tesseract_command_language/core/instruction.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/component_info_impl.h>

int main (int argc, char *argv[])
{
  using namespace tesseract_planning;

  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(6));

  // Define raster posese
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));
  Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.4, 1));
  Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -0.2, 1));
  Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.0, 1));
  Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.2, 1));
  Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, 0.4, 1));

  JointWaypoint* cast_test1 = wp1.cast<JointWaypoint>();
  CartesianWaypoint* cast_test2 = wp2.cast<CartesianWaypoint>();

  // Define raster move instruction
  MoveInstruction move_c1(wp3);
  move_c1.getPointConstraints().push_back(FixedComponent());
  move_c1.getPathConstraints().push_back(LinearComponent());

  MoveInstruction move_c2(wp4);
  move_c2.getPointConstraints().push_back(FixedComponent());
  move_c2.getPathConstraints().push_back(LinearComponent());

  MoveInstruction move_c3(wp5);
  move_c3.getPointConstraints().push_back(FixedComponent());
  move_c3.getPathConstraints().push_back(LinearComponent());

  MoveInstruction move_c4(wp6);
  move_c4.getPointConstraints().push_back(FixedComponent());
  move_c4.getPathConstraints().push_back(LinearComponent());

  MoveInstruction move_c5(wp7);
  move_c5.getPointConstraints().push_back(FixedComponent());
  move_c5.getPathConstraints().push_back(LinearComponent());


  // Create a program
  CompositeInstruction program;

  MoveInstruction move_f0(wp1);
  move_f0.getPointConstraints().push_back(FixedComponent());
  move_f0.getPathConstraints().push_back(FreespaceComponent());
  program.push_back(move_f0);


  MoveInstruction move_f1(wp2);
  move_f1.getPointConstraints().push_back(FixedComponent());
  move_f1.getPathConstraints().push_back(FreespaceComponent());

  CompositeInstruction raster1;
  raster1.push_back(move_f1);
  raster1.push_back(move_c1);
  raster1.push_back(move_c2);
  raster1.push_back(move_c3);
  raster1.push_back(move_c4);
  raster1.push_back(move_c5);
  raster1.getCompositeCosts().push_back(VelocitySmoothingComponent());
  program.push_back(raster1);

  MoveInstruction move_f2(wp2);
  move_f1.getPointConstraints().push_back(FixedComponent());
  move_f1.getPathConstraints().push_back(FreespaceComponent());

  CompositeInstruction raster2;
  raster2.push_back(move_f2);
  raster2.push_back(move_c1);
  raster2.push_back(move_c2);
  raster2.push_back(move_c3);
  raster2.push_back(move_c4);
  raster2.push_back(move_c5);
  raster2.getCompositeCosts().push_back(VelocitySmoothingComponent());
  program.push_back(raster2);

  MoveInstruction move_f3(wp2);
  move_f1.getPointConstraints().push_back(FixedComponent());
  move_f1.getPathConstraints().push_back(FreespaceComponent());

  CompositeInstruction raster3;
  raster3.push_back(move_f3);
  raster3.push_back(move_c1);
  raster3.push_back(move_c2);
  raster3.push_back(move_c3);
  raster3.push_back(move_c4);
  raster3.push_back(move_c5);
  raster3.getCompositeCosts().push_back(VelocitySmoothingComponent());
  program.push_back(raster3);

  program.push_back(move_f3);

//  struct ProgramMetaData
//  {
//    Eigen::VectorXd current_position;
//  };

//  struct Program
//  {
//    ProgramMetaData metadata;
//    CompositeInstruction instructions;
//  };
}
