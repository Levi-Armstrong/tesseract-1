/**
 * @file process_definition_config.h
 * @brief Tesseract process definition configuration
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_PROCESS_PLANNERS_PROCESS_DEFINITION_CONFIG_H
#define TESSERACT_PROCESS_PLANNERS_PROCESS_DEFINITION_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_process_planners/process_transition_generator.h>

namespace tesseract_process_planners
{

/**
 * @class tesseract_process_planners::ProcessDefinitionConfig
 * @details The Process Definition Config
 *
 * This provides the high level process configuration information. It requires the user to provide
 * the start position waypoint (JointWaypoint) and a set of tool paths (strokes) that should be
 * converted into a process results definition leveraging both this configuration information and
 * the ProcessSegmentDefinitions.
 *
 * Also, other operations that are nice to have is the ability to offset the process. Particularly useful
 * when wanting to verify the process without making contact with a surface in the case of sanding.
 */
struct ProcessDefinitionConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  tesseract_motion_planners::Waypoint::Ptr start;
  std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>> tool_paths;

  std::vector<ProcessTransitionGenerator::ConstPtr> transition_generator;

  Eigen::Isometry3d local_offset_direction;
  Eigen::Isometry3d world_offset_direction;

  ProcessDefinitionConfig()
  {
    local_offset_direction.setIdentity();
    world_offset_direction.setIdentity();
  }
};

}
#endif // TESSERACT_PROCESS_PLANNERS_PROCESS_DEFINITION_CONFIG_H
