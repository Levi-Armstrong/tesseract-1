
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#elif not defined(_WIN32)
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#endif

#include <ignition/math/eigen3/Conversions.hh>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/tesseract_viewer.h>
#include <tesseract_visualization/glut_window.h>

using namespace ignition;
using namespace rendering;

namespace tesseract_visualization
{
TesseractViewer::TesseractViewer(tesseract_environment::Environment::Ptr env, const std::string& engine_name)
  : env_(std::move(env))
{
  // create scene
  RenderEngine* engine = rendering::engine(engine_name);
  if (!engine)
  {
    CONSOLE_BRIDGE_logError("Engine '%s' is not supported\n", engine_name.c_str());
    return;
  }
  ign_scene_ = engine->CreateScene("scene");

  // initialize _scene
  ign_scene_->SetAmbientLight(0.3, 0.3, 0.3);
  ign_scene_->SetBackgroundColor(0.3, 0.3, 0.3);
  VisualPtr root = ign_scene_->RootVisual();

  AxisVisualPtr axis = ign_scene_->CreateAxisVisual();
  axis->Scale(0.5, 0.5, 0.5);
  root->AddChild(axis);

  if (env_)
  {
    tesseract_environment::EnvState::ConstPtr state = env_->getCurrentState();
    tesseract_scene_graph::SceneGraph::ConstPtr sg = env_->getSceneGraph();
    for (const auto& link : sg->getLinks())
    {
      VisualPtr ign_link = ign_scene_->CreateVisual(link->getName());
      ign_link->SetWorldPose(ignition::math::eigen3::convert(state->link_transforms.at(link->getName())));
      for (const auto& vs : link->visual)
      {
        switch (vs->geometry->getType())
        {
          case tesseract_geometry::GeometryType::BOX:
          {
            VisualPtr box = ign_scene_->CreateVisual();
            box->SetLocalPose(ignition::math::eigen3::convert(vs->origin));
            box->AddGeometry(ign_scene_->CreateBox());

            auto shape = std::static_pointer_cast<const tesseract_geometry::Box>(vs->geometry);
            box->Scale(shape->getX(), shape->getY(), shape->getZ());
            ign_link->AddChild(box);
            break;
          }
          case tesseract_geometry::GeometryType::SPHERE:
          {
            VisualPtr sphere = ign_scene_->CreateVisual();
            sphere->SetLocalPose(ignition::math::eigen3::convert(vs->origin));
            sphere->AddGeometry(ign_scene_->CreateSphere());

            auto shape = std::static_pointer_cast<const tesseract_geometry::Sphere>(vs->geometry);
            sphere->Scale(shape->getRadius(), shape->getRadius(), shape->getRadius());
            ign_link->AddChild(sphere);
            break;
          }
          case tesseract_geometry::GeometryType::CYLINDER:
          {
            VisualPtr cylinder = ign_scene_->CreateVisual();
            cylinder->SetLocalPose(ignition::math::eigen3::convert(vs->origin));
            cylinder->AddGeometry(ign_scene_->CreateCylinder());

            auto shape = std::static_pointer_cast<const tesseract_geometry::Cylinder>(vs->geometry);
            cylinder->Scale(shape->getRadius(), shape->getRadius(), shape->getLength());
            ign_link->AddChild(cylinder);
            break;
          }
          case tesseract_geometry::GeometryType::CONE:
          {
            VisualPtr cone = ign_scene_->CreateVisual();
            cone->SetLocalPose(ignition::math::eigen3::convert(vs->origin));
            cone->AddGeometry(ign_scene_->CreateCone());

            auto shape = std::static_pointer_cast<const tesseract_geometry::Cone>(vs->geometry);
            cone->Scale(shape->getRadius(), shape->getRadius(), shape->getLength());
            ign_link->AddChild(cone);
            break;
          }
          case tesseract_geometry::GeometryType::CAPSULE:
          {
            //          VisualPtr capsule = ign_scene_->CreateVisual();
            //          capsule->SetLocalPose(ignition::math::eigen3::convert(vs->origin));
            //          capsule->AddGeometry(ign_scene_->CreateCapsule());

            //          auto shape = std::static_pointer_cast<const tesseract_geometry::Capsule>(vs->geometry);
            //          capsule->Scale(shape->getRadius(), shape->getRadius(), shape->getLength());
            //          ign_link->AddChild(capsule);
            break;
          }
          case tesseract_geometry::GeometryType::MESH:
          {
            auto shape = std::static_pointer_cast<const tesseract_geometry::Mesh>(vs->geometry);
            auto resource = shape->getResource();
            if (resource)
            {
              VisualPtr mesh = ign_scene_->CreateVisual();
              mesh->SetLocalPose(ignition::math::eigen3::convert(vs->origin));

              MeshDescriptor descriptor;
              descriptor.meshName = resource->getFilePath();
              common::MeshManager* mesh_manager = common::MeshManager::Instance();
              descriptor.mesh = mesh_manager->Load(descriptor.meshName);
              MeshPtr mesh_geom = ign_scene_->CreateMesh(descriptor);
              mesh->AddGeometry(mesh_geom);
              ign_link->AddChild(mesh);
            }
            else
            {
              assert(false);
            }

            break;
          }
          case tesseract_geometry::GeometryType::CONVEX_MESH:
          {
            auto shape = std::static_pointer_cast<const tesseract_geometry::ConvexMesh>(vs->geometry);
            auto resource = shape->getResource();
            if (resource)
            {
              VisualPtr mesh = ign_scene_->CreateVisual();
              mesh->SetLocalPose(ignition::math::eigen3::convert(vs->origin));

              MeshDescriptor descriptor;
              descriptor.meshName = resource->getFilePath();
              common::MeshManager* mesh_manager = common::MeshManager::Instance();
              descriptor.mesh = mesh_manager->Load(descriptor.meshName);
              MeshPtr mesh_geom = ign_scene_->CreateMesh(descriptor);
              mesh->AddGeometry(mesh_geom);
              ign_link->AddChild(mesh);
            }
            else
            {
              assert(false);
            }

            break;
          }
          case tesseract_geometry::GeometryType::OCTREE:
          {
            auto shape = std::static_pointer_cast<const tesseract_geometry::Octree>(vs->geometry);

            // TODO: Need to implement
            assert(false);
            break;
          }
          default:
          {
            CONSOLE_BRIDGE_logError("This geometric shape type (%d) is not supported",
                                    static_cast<int>(vs->geometry->getType()));
            break;
          }
        }
      }
      root->AddChild(ign_link);
    }
  }

  // create gray material
  MaterialPtr gray = ign_scene_->CreateMaterial();
  gray->SetAmbient(0.7, 0.7, 0.7);
  gray->SetDiffuse(0.7, 0.7, 0.7);
  gray->SetSpecular(0.7, 0.7, 0.7);

  // create grid visual
  GridPtr gridGeom = ign_scene_->CreateGrid();
  if (gridGeom)
  {
    VisualPtr grid = ign_scene_->CreateVisual();
    gridGeom->SetCellCount(20);
    gridGeom->SetCellLength(1);
    gridGeom->SetVerticalCellCount(0);
    grid->AddGeometry(gridGeom);
    grid->SetLocalPosition(3, 0, 0.0);
    grid->SetMaterial(gray);
    root->AddChild(grid);
  }

  // create camera
  camera_ = ign_scene_->CreateCamera("camera");
  camera_->SetLocalPosition(0.0, 0.0, 0.5);
  camera_->SetLocalRotation(0.0, 0.0, 0.0);
  camera_->SetImageWidth(800);
  camera_->SetImageHeight(600);
  camera_->SetAntiAliasing(2);
  camera_->SetAspectRatio(1.333);
  camera_->SetHFOV(IGN_PI / 2);

  // create directional light
  DirectionalLightPtr light0 = ign_scene_->CreateDirectionalLight();
  light0->SetDirection(1, 0, 0);
  light0->SetDiffuseColor(0.8, 0.8, 0.8);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  camera_->AddChild(light0);

  root->AddChild(camera_);
}

void TesseractViewer::update() {}

void TesseractViewer::show()
{
  if (!is_shown_)
  {
    is_shown_ = true;
    //    window_thread_ = boost::thread(&run, {camera_}, nullptr, nullptr);
    run({ camera_ }, nullptr, nullptr);
  }
}
}  // namespace tesseract_visualization
