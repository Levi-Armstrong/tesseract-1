/**
 * @file process_transition_generator.h
 * @brief Tesseract process transition generator interface
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
#ifndef TESSERACT_PROCESS_PLANNERS_PROCESS_TRANSITION_GENERATOR_H
#define TESSERACT_PROCESS_PLANNERS_PROCESS_TRANSITION_GENERATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_process_planners
{

/**
 * @class tesseract_process_planners::ProcessTransitionGenerator
 */
class ProcessTransitionGenerator
{
public:
  using Ptr = std::shared_ptr<ProcessTransitionGenerator>;
  using ConstPtr = std::shared_ptr<const ProcessTransitionGenerator>;

  virtual ~ProcessTransitionGenerator() = default;
  virtual std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const tesseract_motion_planners::Waypoint::Ptr& start_waypoint,
           const tesseract_motion_planners::Waypoint::Ptr& end_waypoint) const = 0;
};

}

#endif // TESSERACT_PROCESS_PLANNERS_PROCESS_TRANSITION_GENERATOR_H
