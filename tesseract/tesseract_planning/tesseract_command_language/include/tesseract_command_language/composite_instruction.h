#ifndef TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/instruction_type.h>
#include <vector>

namespace tesseract_planning
{

enum class CompositeInstructionOrder
{
  ORDERED, // Must go in forward
  UNORDERED, // Any order is allowed
  ORDERED_AND_INVERTABLE // Can go forward or reverse the order
};

class CompositeInstruction : public std::vector<Instruction>
{
public:
  using Ptr = std::shared_ptr<CompositeInstruction>;
  using ConstPtr = std::shared_ptr<const CompositeInstruction>;

  CompositeInstruction(CompositeInstructionOrder order = CompositeInstructionOrder::ORDERED);

  CompositeInstructionOrder getOrder() const;

  std::vector<ComponentInfo>& getCompositeCosts();
  const std::vector<ComponentInfo>& getCompositeCosts() const;

  std::vector<ComponentInfo>& getCompositeConstraints();
  const std::vector<ComponentInfo>& getCompositeConstraints() const;

  int getType() const;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  bool isComposite() const;

  bool isMove() const;

  void print() const;

  CompositeInstruction flatten() const;

private:
  int type_ { static_cast<int>(InstructionType::COMPOSITE_INSTRUCTION) };

  std::string description_;

  std::vector<ComponentInfo> composite_costs_;

  std::vector<ComponentInfo> composite_constraints_;

  CompositeInstructionOrder order_;

  void flattenHelper(CompositeInstruction& flattened, const CompositeInstruction& composite) const;

};

}

#endif // TESSERACT_COMMAND_LANGUAGE_COMPOSITE_INSTRUCTION_H
