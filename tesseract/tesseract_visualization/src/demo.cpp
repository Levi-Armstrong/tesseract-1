#include <tesseract_visualization/tesseract_viewer.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_environment/kdl/kdl_env.h>

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

tesseract_scene_graph::SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

int main(int _argc, char** _argv)
{
  auto env = std::make_shared<tesseract_environment::KDLEnv>();
  ;

  if (env->init(getSceneGraph()))
  {
    tesseract_visualization::TesseractViewer viewer(env, "ogre");
    viewer.show();
  }

  return 0;
}
