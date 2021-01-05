/**
 * @file tool_pickup_taskflow.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/utils.h>
#include <tesseract_process_managers/taskflow_generators/tool_pickup_taskflow.h>

#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

ToolPickupTaskflow::ToolPickupTaskflow(TaskflowGenerator::UPtr global_taskflow_generator,
                                       TaskflowGenerator::UPtr freespace_taskflow_generator,
                                       TaskflowGenerator::UPtr cartesian_taskflow_generator,
                                       std::string name)
  : global_taskflow_generator_(std::move(global_taskflow_generator))
  , freespace_taskflow_generator_(std::move(freespace_taskflow_generator))
  , cartesian_taskflow_generator_(std::move(cartesian_taskflow_generator))
  , name_(name)
{
}

const std::string& ToolPickupTaskflow::getName() const { return name_; }

TaskflowContainer ToolPickupTaskflow::generateTaskflow(ProcessInput input,
                                                       TaskflowVoidFn done_cb,
                                                       TaskflowVoidFn error_cb)
{
  // This should make all of the isComposite checks so that you can safely cast below
  if (!checkProcessInput(input))
  {
    CONSOLE_BRIDGE_logError("Invalid Process Input");
    throw std::runtime_error("Invalid Process Input");
  }

  TaskflowContainer container;
  container.taskflow = std::make_unique<tf::Taskflow>(name_);

  const Instruction* input_instruction = input.getInstruction();
  TaskflowContainer sub_container = global_taskflow_generator_->generateTaskflow(
      input,
      [=]() { successTask(input, name_, input_instruction->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, input_instruction->getDescription(), error_cb); });

  auto global_task = container.taskflow->composed_of(*(sub_container.taskflow)).name("global");
  container.containers.push_back(std::move(sub_container));
  container.input = global_task;

  auto global_post_task = container.taskflow->emplace([=]() { globalPostProcess(input); }).name("global post process");
  global_task.precede(global_post_task);

  // Cartesian Approach
  ProcessInput cartesian_input = input[1];
  cartesian_input.setStartInstruction(input_instruction->cast_const<CompositeInstruction>()[0][0]);
  TaskflowContainer cartesian_container = cartesian_taskflow_generator_->generateTaskflow(
      cartesian_input,
      [=]() { successTask(input, name_, cartesian_input.getInstruction()->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, cartesian_input.getInstruction()->getDescription(), error_cb); });

  auto cartesian_step = container.taskflow->composed_of(*(cartesian_container.taskflow))
                            .name("Cartesian Approach: " + cartesian_input.getInstruction()->getDescription());
  container.containers.push_back(std::move(cartesian_container));
  global_post_task.precede(cartesian_step);

  // Freespace
  ProcessInput freespace_input = input[0];
  freespace_input.setStartInstruction(input_instruction->cast_const<CompositeInstruction>()->getStartInstruction());
  freespace_input.setEndInstruction(std::vector<std::size_t>({ 1 }));
  TaskflowContainer freespace_container = freespace_taskflow_generator_->generateTaskflow(
      freespace_input,
      [=]() { successTask(input, name_, freespace_input.getInstruction()->getDescription(), done_cb); },
      [=]() { failureTask(input, name_, freespace_input.getInstruction()->getDescription(), error_cb); });

  auto freespace_step = container.taskflow->composed_of(*(freespace_container.taskflow))
                            .name("Freespace: " + freespace_input.getInstruction()->getDescription());
  container.containers.push_back(std::move(freespace_container));
  cartesian_step.precede(freespace_step);

  return container;
}

void ToolPickupTaskflow::globalPostProcess(ProcessInput input)
{
  if (input.isAborted())
    return;

  CompositeInstruction* results = input.getResults()->cast<CompositeInstruction>();
  CompositeInstruction* composite = results->at(0).cast<CompositeInstruction>();
  composite->setStartInstruction(results->getStartInstruction());
  composite->setManipulatorInfo(results->getManipulatorInfo());
  for (std::size_t i = 1; i < results->size(); ++i)
  {
    CompositeInstruction* composite0 = results->at(i - 1).cast<CompositeInstruction>();
    MoveInstruction lmi = *getLastMoveInstruction(*composite0);
    lmi.setMoveType(MoveInstructionType::START);

    CompositeInstruction* composite1 = results->at(i).cast<CompositeInstruction>();
    composite1->setStartInstruction(lmi);
    composite1->setManipulatorInfo(results->getManipulatorInfo());
  }
}

bool ToolPickupTaskflow::checkProcessInput(const tesseract_planning::ProcessInput& input) const
{
  // -------------
  // Check Input
  // -------------
  if (!input.env)
  {
    CONSOLE_BRIDGE_logError("ProcessInput env is a nullptr");
    return false;
  }

  // Check the overall input
  const Instruction* input_instruction = input.getInstruction();
  if (!isCompositeInstruction(*input_instruction))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: input.instructions should be a composite");
    return false;
  }
  const auto* composite = input_instruction->cast_const<CompositeInstruction>();

  // Check that it has a start instruction
  if (!composite->hasStartInstruction() && isNullInstruction(input.getStartInstruction()))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: input.instructions should have a start instruction");
    return false;
  }

  // Check size of composite, it should only contain two entries
  if (composite->size() != 2)
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: The program should have two composite instruction.");
    return false;
  }

  // Check first instruction should be a composite instruction with a single PlanInstruction of type freespace
  if (!isCompositeInstruction(composite->at(0)))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: The first instruction should be a CompositeInstruction!");
    return false;
  }
  else
  {
    const auto* freespace = composite->at(0).cast_const<CompositeInstruction>();
    if (freespace->size() != 1)
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: The first instruction should be a Composite with a single "
                              "PlanInstruction of type freespace!");
      return false;
    }
    else if (freespace->at(0).cast_const<PlanInstruction>()->getPlanType() != PlanInstructionType::FREESPACE)
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: The first instruction should be a Composite with a single "
                              "PlanInstruction of type freespace!");
      return false;
    }
  }

  // Check second instruction should be a composite instruction with a single plan instruction of type linear
  if (!isCompositeInstruction(composite->at(1)))
  {
    CONSOLE_BRIDGE_logError("ProcessInput Invalid: The second instruction should be a CompositeInstruction!");
    return false;
  }
  else
  {
    const auto* cartesian = composite->at(1).cast_const<CompositeInstruction>();
    if (cartesian->size() != 1)
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: The second instruction should be a Composite with a single "
                              "PlanInstruction of type linear!");
      return false;
    }
    else if (cartesian->at(1).cast_const<PlanInstruction>()->getPlanType() != PlanInstructionType::LINEAR)
    {
      CONSOLE_BRIDGE_logError("ProcessInput Invalid: The second instruction should be a Composite with a single "
                              "PlanInstruction of type linear!");
      return false;
    }
  }
  return true;
}
