/**
 * @file tool_pickup_taskflow.h
 * @brief A tool pickup taskflow which assumes the first instructions is a freespace plan instruction followed by a
 * linear plan instruction.
 *
 * @author Levi Armstrong
 * @date January 4, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_PROCESS_MANAGERS_TOOL_PICKUP_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_TOOL_PICKUP_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <vector>
#include <thread>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/taskflow_generator.h>

namespace tesseract_planning
{
/**
 * @brief This class provides a process manager for a tool pickup process.
 *
 * Given a ProcessInput in the correct format, it handles the creation of the process dependencies and uses Taskflow to
 * execute them efficiently in a parallel based on those dependencies.
 *
 * The required format is below.
 *
 * Composite
 * {
 *   Composite - Contains Single PlanInstruction of type Freespace
 *   Composite - Contains Single PlanInstruction of type Linear
 * }
 */
class ToolPickupTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<ToolPickupTaskflow>;

  ToolPickupTaskflow(TaskflowGenerator::UPtr global_taskflow_generator,
                     TaskflowGenerator::UPtr freespace_taskflow_generator,
                     TaskflowGenerator::UPtr cartesian_taskflow_generator,
                     std::string name = "ToolPickupTaskflow");

  ~ToolPickupTaskflow() override = default;
  ToolPickupTaskflow(const ToolPickupTaskflow&) = delete;
  ToolPickupTaskflow& operator=(const ToolPickupTaskflow&) = delete;
  ToolPickupTaskflow(ToolPickupTaskflow&&) = delete;
  ToolPickupTaskflow& operator=(ToolPickupTaskflow&&) = delete;

  const std::string& getName() const override;

  TaskflowContainer generateTaskflow(ProcessInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb) override;

private:
  TaskflowGenerator::UPtr global_taskflow_generator_;
  TaskflowGenerator::UPtr freespace_taskflow_generator_;
  TaskflowGenerator::UPtr cartesian_taskflow_generator_;
  std::string name_;

  static void globalPostProcess(ProcessInput input);

  /**
   * @brief Checks that the ProcessInput is in the correct format.
   * @param input ProcessInput to be checked
   * @return True if in the correct format
   */
  bool checkProcessInput(const ProcessInput& input) const;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_TOOL_PICKUP_TASKFLOW_H
