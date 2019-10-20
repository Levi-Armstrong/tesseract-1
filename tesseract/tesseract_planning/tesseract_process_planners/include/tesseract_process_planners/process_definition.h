/**
 * @file process_definition.h
 * @brief Tesseract process definition
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
#ifndef TESSERACT_PLANNING_PROCESS_DEFINITION_H
#define TESSERACT_PLANNING_PROCESS_DEFINITION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_process_planners/process_definition_config.h>
#include <tesseract_process_planners/process_segment_definition.h>
#include <tesseract_process_planners/process_segment_definition_config.h>
#include <tesseract_process_planners/process_transition_definition.h>

namespace tesseract_process_planners
{

/**
 * @class tesseract_process_planners::ProcessDefinition
 * @details A process definition.
 *
 * This is not intended to support or handle all processes but should covers most of the common ones
 * like sanding, painting, grinding, etc.
 *
 * In a process, we assume they follow this pattern.
 *    * The robot starts from a start position,
 *    * Moves to an approach position just above the object
 *    * Executes a process(sanding, painting, etc.)
 *    * Return to the start position
 *
 * Given the process described the user is only required to define two objects. The start position of
 * the robot and a vector of Process Segment Definitions.
 */
class ProcessDefinition
{
public:
  class Iterator;
  /**
   * @brief The start position of the robot
   */
  tesseract_motion_planners::Waypoint::Ptr start;

  /**
   * @brief All of the raster segments with approaches and departures
   */
  std::vector<ProcessSegmentDefinition> segments;

  /**
   * @brief All of the transition to/from a given segment. Must be same length as segments
   */
  std::vector<ProcessTransitionDefinition> transitions;

  static ProcessDefinition generate(const ProcessDefinitionConfig& process_config,
                                    const ProcessSegmentDefinitionConfig& segment_config);

  static ProcessDefinition generate(const ProcessDefinitionConfig& process_config,
                                    const std::vector<ProcessSegmentDefinitionConfig>& segment_config);

  /**
   * @brief Get the size = approach.size() + process.size() + departure.size()
   * @return The size of the process segement definition
   */
  std::size_t size() const;

  /**
   * @brief Get the iterator to the beginning of the process segment definition
   * @return Iterator
   */
  Iterator begin();

  /**
   * @brief Get the iterator to the end of the process segment definition
   * @return Iterator
   */
  Iterator end();

  /**
   * @brief Get waypoint by index
   * @param index The index
   * @return Waypoint
   */
  tesseract_motion_planners::Waypoint::Ptr& operator[](std::size_t index);

  /**
   * @brief Get const waypoint by index
   * @param index The index
   * @return Const Waypoint
   */
  const tesseract_motion_planners::Waypoint::Ptr& operator[](std::size_t index) const;

  /**
   * @brief Erase waypoint given iterator
   * @param pos The iterator associated with the waypoint to remove
   * @return The iterator
   */
  Iterator erase(Iterator pos);

  /**
   * @brief Erase waypoints given iterator
   * @param pos The iterator associated with the waypoint to remove
   * @return The iterator
   */
  Iterator erase(Iterator first, Iterator last);

  /** @brief The Iterator class ProcessSegmentDefinition */
  class Iterator : public std::iterator<std::random_access_iterator_tag, tesseract_motion_planners::Waypoint::Ptr>
  {
  public:

    Iterator(ProcessDefinition& container, std::size_t pos);

    tesseract_motion_planners::Waypoint::Ptr& operator* ();

    const tesseract_motion_planners::Waypoint::Ptr& operator* () const;

    const Iterator& operator++ ();

    Iterator operator++ (int);

    const Iterator& operator-- ();

    Iterator operator-- (int);

    difference_type operator-(const Iterator& rhs) const;

    Iterator operator+(difference_type rhs) const;

    Iterator operator-(difference_type rhs) const;

    bool operator==(const Iterator& rhs) const;

    bool operator!=(const Iterator& rhs) const;

    bool operator>(const Iterator& rhs) const;

    bool operator<(const Iterator& rhs) const;

    bool operator>=(const Iterator& rhs) const;

    bool operator<=(const Iterator& rhs) const;

  private:
    //https://stackoverflow.com/questions/1784573/iterator-for-2d-vector
    std::size_t segment_idx_outer = 0;
    std::size_t segment_idx_inner = 0; // This will also include transition to not have to add another indx
    ProcessDefinition* container_;
  };
};

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PLANNING_PROCESS_DEFINITION_H
