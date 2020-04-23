/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef TESSERACT_VISUALIZATION_GLUT_WINDOW_H
#define TESSERACT_VISUALIZATION_GLUT_WINDOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/common/graphics/Types.hh>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_visualization
{
/// \brief Run the demo and render the scene from the cameras
/// \param[in] _cameras Cameras in the scene
/// \param[in] _mesh Actor mesh
/// \param[in] _skel Actor skeleton
void run(std::vector<ignition::rendering::CameraPtr> cameras,
         ignition::rendering::MeshPtr mesh,
         ignition::common::SkeletonPtr skel);

}  // namespace tesseract_visualization
#endif
