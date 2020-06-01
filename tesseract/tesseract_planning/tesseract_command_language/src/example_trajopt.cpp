#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/component_info.h>
#include <tesseract_command_language/core/instruction.h>

#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/component_info_impl.h>

#include <tesseract_command_language/trajopt_planner_universal_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>


std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

tesseract_planning::CompositeInstruction generateSeed(const tesseract_planning::CompositeInstruction& instructions)
{
  tesseract_planning::CompositeInstruction seed;
  const tesseract_planning::PlanInstruction* prev_plan_instruction {nullptr};

  for (const auto& instruction : instructions)
  {
    if (instruction.isPlan())
    {
      const tesseract_planning::PlanInstruction* plan_instruction = instruction.cast_const<tesseract_planning::PlanInstruction>();
      if (plan_instruction->isLinear())
      {
        if (prev_plan_instruction)
        {

        }
        else
        {

        }
      }
      else if (plan_instruction->isFreespace())
      {

      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }
    }
    else
    {
      seed.push_back(instruction);
    }
  }
  return seed;
}

int main (int argc, char *argv[])
{
  using namespace tesseract_planning;

  // Start Joint Position for the program
  Waypoint wp1 = JointWaypoint(Eigen::VectorXd::Ones(6));

  // Define goal pose
  Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(1, -1, 1));


  // Define freespace move instruction
  PlanInstruction plan_f1(wp2, PlanInstructionType::FREESPACE);
  plan_f1.addConstraint(FixedComponent());

  // Create a program
  CompositeInstruction program;
  program.push_back(plan_f1);
  program.addCost(VelocitySmoothingComponent());
  program.addCost(AccelerationSmoothingComponent());
  program.addCost(JerkSmoothingComponent());

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  tesseract->init(urdf_path, srdf_path, locator);

  auto config = std::make_shared<tesseract_planning::TrajOptPlannerUniversalConfig>(tesseract, "manipulator", "tool0", Eigen::Isometry3d::Identity());
  config->instructions = program;

  tesseract_motion_planners::TrajOptMotionPlanner planner;
  planner.setConfiguration(config);

  tesseract_motion_planners::PlannerResponse response;

  auto status = planner.solve(response);

}
