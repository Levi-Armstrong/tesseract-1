/**
 * @file process_step_generator.h
 * @brief Tesseract process step generator interface
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
#ifndef TESSERACT_PROCESS_PLANNERS_PROCESS_STEP_GENERATOR_H
#define TESSERACT_PROCESS_PLANNERS_PROCESS_STEP_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_process_planners/process_definition_config.h>

namespace tesseract_process_planners
{

/**
 * @class tesseract_process_planners::ProcessStepGenerator
 * @details
 * This is a base class for process step generators. A process step generator takes tool path segments and
 * converts them into process tool path. Currently we assume that each tool path segment has three
 * steps Approach, Process, and Departure. Example case is using raster tool paths on surfaces but these need
 * to be modified based on the process that will be using those tool paths. Custom implementations of this class
 *  could for instance not want to stay normal to the surface but apply an angle of attack instead.
 * Therefore the specialized implementation would take the tool path segments and generate a new segments with
 * modified poses that accommodate the angle of attack. This class aims to bridge the gap between surface rastering
 * libraries and the planners so neither of these library need to know anything about the particular process.
 */
class ProcessStepGenerator
{
public:
  using Ptr = std::shared_ptr<ProcessStepGenerator>;
  using ConstPtr = std::shared_ptr<const ProcessStepGenerator>;

  virtual ~ProcessStepGenerator() = default;

  virtual std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
           const ProcessDefinitionConfig& config) const = 0;
};

} // tesseract_process_planners

#endif // TESSERACT_PROCESS_PLANNERS_PROCESS_STEP_GENERATOR_H
