#include <iostream>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_model_linked/model.h>

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

    mod_url = package_path + mod_url;  // "file://" + package_path + mod_url;
  }

  return mod_url;
}

tesseract_scene_graph::SceneGraph::UPtr getSceneGraphIIWA()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

tesseract_scene_graph::SceneGraph::UPtr getSceneGraphUR()
{
  using namespace tesseract_scene_graph;

  auto sg = std::make_unique<SceneGraph>("universal_robot");
  sg->addLink(Link("ur_base_link"));
  sg->addLink(Link("ur_shoulder_link"));
  sg->addLink(Link("ur_upper_arm_link"));
  sg->addLink(Link("ur_forearm_link"));
  sg->addLink(Link("ur_wrist_1_link"));
  sg->addLink(Link("ur_wrist_2_link"));
  sg->addLink(Link("ur_wrist_3_link"));
  sg->addLink(Link("ur_ee_link"));
  sg->addLink(Link("ur_tool0"));

  {
    Joint j("ur_shoulder_pan_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "ur_base_link";
    j.child_link_name = "ur_shoulder_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0.1273);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ur_shoulder_lift_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "ur_shoulder_link";
    j.child_link_name = "ur_upper_arm_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0.220941, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ur_elbow_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "ur_upper_arm_link";
    j.child_link_name = "ur_forearm_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, -0.1719, 0.612);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ur_wrist_1_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "ur_forearm_link";
    j.child_link_name = "ur_wrist_1_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0.5723);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ur_wrist_2_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "ur_wrist_1_link";
    j.child_link_name = "ur_wrist_2_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0.163941 + 0.1719 - 0.220941, 0);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ur_wrist_3_joint");
    j.type = JointType::REVOLUTE;
    j.parent_link_name = "ur_wrist_2_link";
    j.child_link_name = "ur_wrist_3_link";
    j.axis = Eigen::Vector3d::UnitY();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0, 0.1157);
    j.limits = std::make_shared<JointLimits>();
    j.limits->lower = -2.0 * M_PI;
    j.limits->upper = 2.0 * M_PI;
    j.limits->velocity = 2.16;
    j.limits->acceleration = 0.5 * j.limits->velocity;
    sg->addJoint(j);
  }

  {
    Joint j("ur_ee_fixed_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "ur_wrist_3_link";
    j.child_link_name = "ur_ee_link";
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0.0922, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
    sg->addJoint(j);
  }

  {
    Joint j("ur_wrist_3_link-tool0_fixed_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "ur_wrist_3_link";
    j.child_link_name = "ur_tool0";
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 0.0922, 0);
    j.parent_to_joint_origin_transform =
        j.parent_to_joint_origin_transform * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
    sg->addJoint(j);
  }

  return sg;
}

tesseract_scene_graph::SceneGraph::UPtr getWorldSceneGraph()
{
  using namespace tesseract_scene_graph;

  auto sg = std::make_unique<SceneGraph>("world");
  sg->addLink(Link("world"));
  sg->addLink(Link("ur_attach_link"));
  sg->addLink(Link("iiwa_attach_link"));

  {
    Joint j("ur_attach_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "world";
    j.child_link_name = "ur_attach_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(1, 0, 0);
    sg->addJoint(j);
  }

  {
    Joint j("iiwa_attach_joint");
    j.type = JointType::FIXED;
    j.parent_link_name = "world";
    j.child_link_name = "iiwa_attach_link";
    j.axis = Eigen::Vector3d::UnitZ();
    j.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0, 1, 0);
    sg->addJoint(j);
  }
  return sg;
}

int main(int /*argc*/, char** /*argv*/)
{
  using namespace tesseract_model_linked;
  using namespace tesseract_scene_graph;

  SceneGraph::UPtr world_sg = getWorldSceneGraph();
  SceneGraph::UPtr iiwa_sg = getSceneGraphIIWA();
  SceneGraph::UPtr ur_sg = getSceneGraphUR();

  auto world_model = std::make_unique<Model>(std::move(world_sg));
  auto iiwa_model = std::make_unique<Model>(std::move(iiwa_sg));
  auto ur_model = std::make_unique<Model>(std::move(ur_sg));

  std::string world_model_name = world_model->getName();
  std::string iiwa_model_name = iiwa_model->getName();
  std::string ur_model_name = ur_model->getName();

  world_model->addChild(std::move(iiwa_model), "iiwa_attach_link");
  world_model->addChild(std::move(ur_model), "ur_attach_link");

  ModelState::UPtr state = world_model->getState();
  VariableSets variable_sets = state->getVariableSets();
  VariableSet iiwa_variables = variable_sets[iiwa_model_name];

  std::cout << "Model Name: " << world_model->getName() << std::endl;

  std::cout << "Link Names: " << std::endl;
  for (const auto& link_name : world_model->getLinkNames(true))
    std::cout << "    " << link_name << std::endl;
  std::cout << std::endl;

  std::cout << "Active Link Names: " << std::endl;
  for (const auto& link_name : world_model->getActiveLinkNames(true))
    std::cout << "    " << link_name << std::endl;
  std::cout << std::endl;

  std::cout << "Joint Names: " << std::endl;
  for (const auto& joint_name : world_model->getJointNames(true))
    std::cout << "    " << joint_name << std::endl;
  std::cout << std::endl;

  std::cout << "Active Joint Names: " << std::endl;
  for (const auto& joint_name : world_model->getActiveJointNames(true))
    std::cout << "    " << joint_name << std::endl;
  std::cout << std::endl;
}
