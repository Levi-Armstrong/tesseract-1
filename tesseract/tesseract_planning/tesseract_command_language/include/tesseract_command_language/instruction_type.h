#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H

#include <tesseract_command_language/core/instruction.h>

namespace tesseract_planning
{

enum class InstructionType : int
{
  //Everything before must be a motion Instruction
  MOVE_INSTRUCTION,

  // Everything before must be a composite Instruction
  COMPOSITE_INSTRUCTION,

  // Everything before must be a I/O Instruction
  IO_INSTRUCTION,

  // Everything before must be a Analog Instruction
  ANALOG_INSTRUCTION,

  // Everything before must be a variable Instruction
  VARIABLE_INSTRUCTION,

  // Everything before must be a comment Instruction
  COMMENT_INSTRUCTION,

  // User defined types must be larger than this
  USER_DEFINED = 1000
};

inline bool isCommentInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::COMMENT_INSTRUCTION) && instruction.getType() > static_cast<int>(InstructionType::VARIABLE_INSTRUCTION));
}

inline bool isVariableInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::VARIABLE_INSTRUCTION) && instruction.getType() > static_cast<int>(InstructionType::ANALOG_INSTRUCTION));
}

inline bool isAnalogInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::ANALOG_INSTRUCTION) && instruction.getType() > static_cast<int>(InstructionType::IO_INSTRUCTION));
}

inline bool isIOInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::IO_INSTRUCTION) && instruction.getType() > static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION));
}

inline bool isCompositeInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) && instruction.getType() > static_cast<int>(InstructionType::MOVE_INSTRUCTION));
}

inline bool isMoveInstruction(const Instruction& instruction)
{
  return (instruction.getType() <= static_cast<int>(InstructionType::MOVE_INSTRUCTION));
}

}

#endif // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_TYPE_H
