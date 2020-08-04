#include <tesseract_process_managers/process_generators/random_process_generator.h>

namespace tesseract_planning
{

std::function<void()> RandomProcessGenerator::generateTask(ProcessInput input,
                                                           const Instruction &start_instruction,
                                                           const Instruction &end_instruction)
{
  return std::bind(&RandomProcessGenerator::process, this, input, start_instruction, end_instruction);
}

std::function<int()> RandomProcessGenerator::generateConditionalTask(ProcessInput input,
                                                                     const Instruction& start_instruction,
                                                                     const Instruction& end_instruction)
{
  return std::bind(
      &RandomProcessGenerator::conditionalProcess, this, input, start_instruction, end_instruction);
}

void RandomProcessGenerator::process(ProcessInput /*results*/,
                                     const Instruction& /*start_instruction*/,
                                     const Instruction& /*end_instruction*/) const
{
  std::cout << name + "\n";
}

int RandomProcessGenerator::conditionalProcess(ProcessInput /*results*/,
                                               const Instruction& /*start_instruction*/,
                                               const Instruction& /*end_instruction*/) const
{
  Eigen::MatrixX2d limits(1, 2);
  limits << 0, 1;
  Eigen::VectorXd rand = tesseract_common::generateRandomNumber(limits);

  int success = (rand[0] > success_frequency) ? 0 : 1;
  std::cout << name + "  Success: " + std::to_string(success) + "\n";
  return success;
}

bool RandomProcessGenerator::getAbort() const { return abort_; }
void RandomProcessGenerator::setAbort(bool abort) { abort_ = abort; }
}  // namespace tesseract_planning
