#ifndef TESSERACT_VISUALIZATION_TESSERACT_VIEWER_H
#define TESSERACT_VISUALIZATION_TESSERACT_VIEWER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ignition/common/Skeleton.hh>
#include <ignition/common/SkeletonAnimation.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/rendering.hh>
#include <boost/thread/thread.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
namespace tesseract_visualization
{
class TesseractViewer
{
public:
  TesseractViewer(tesseract_environment::Environment::Ptr env, const std::string& engine_name = "ogre2");

  /** @brief This is used to update the scene if the environment changed */
  void update();

  /** @brief Show the viewer window */
  void show();

private:
  tesseract_environment::Environment::Ptr env_;
  ignition::rendering::ScenePtr ign_scene_;
  ignition::rendering::CameraPtr camera_;
  bool is_shown_{ false };
  boost::thread window_thread_;
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_TESSERACT_VIEWER_H
