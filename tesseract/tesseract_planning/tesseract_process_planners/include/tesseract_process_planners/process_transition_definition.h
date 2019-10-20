/**
 * @file process_transition_definition.h
 * @brief Tesseract process transition definition
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
#ifndef TESSERACT_PROCESS_PLANNERS_PROCESS_TRANSITION_DEFINITION_H
#define TESSERACT_PROCESS_PLANNERS_PROCESS_TRANSITION_DEFINITION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_process_planners
{

/**
 * @brief The ProcessTransitionDefinition struct which contains the waypoint data to allow moving between adjacent
 * process segments
 */
struct ProcessTransitionDefinition
{
  /**
   * @brief A transition plans from the start of segment[i] to the end of segment[i+1], this data can be used for finding collision free exit moves after cancelling an ongoing process
   */
  std::vector<tesseract_motion_planners::Waypoint::Ptr> transition_from_start;

  /**
   * @brief A transition plans from the end of segment[i] to the start of segment[i+1]
   */
  std::vector<tesseract_motion_planners::Waypoint::Ptr> transition_from_end;
};

}

#endif // TESSERACT_PROCESS_PLANNERS_PROCESS_TRANSITION_DEFINITION_H
