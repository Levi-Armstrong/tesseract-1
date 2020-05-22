#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{
CompositeInstruction::CompositeInstruction(CompositeInstructionOrder order) : order_(order) {}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

std::vector<ComponentInfo>& CompositeInstruction::getCompositeCosts() { return composite_costs_; }
const std::vector<ComponentInfo>& CompositeInstruction::getCompositeCosts() const { return composite_costs_; }

std::vector<ComponentInfo>& CompositeInstruction::getCompositeConstraints() { return composite_constraints_; }
const std::vector<ComponentInfo>& CompositeInstruction::getCompositeConstraints() const { return composite_constraints_; }

int CompositeInstruction::getType() const { return type_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

bool CompositeInstruction::isComposite() const { return true; }

bool CompositeInstruction::isMove() const { return false; }

void CompositeInstruction::print() const  {}

CompositeInstruction CompositeInstruction::flatten() const
{
  CompositeInstruction flattened;
  flattenHelper(flattened, *this);
  return flattened;
}

void CompositeInstruction::flattenHelper(CompositeInstruction& flattened, const CompositeInstruction& composite) const
{
  for (const auto& i : composite)
  {
    if (i.isComposite())
      flattenHelper(flattened, *(i.cast_const<CompositeInstruction>()));
    else
      flattened.push_back(i);
  }
}

}
