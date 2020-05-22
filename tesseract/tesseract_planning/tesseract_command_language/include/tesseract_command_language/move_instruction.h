#ifndef TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H

#include <tesseract_command_language/core/component_info.h>
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/instruction_type.h>
#include <vector>

namespace tesseract_planning
{

class MoveInstruction
{
public:
  using Ptr = std::shared_ptr<MoveInstruction>;
  using ConstPtr = std::shared_ptr<const MoveInstruction>;

  MoveInstruction(Waypoint waypoint);

  Waypoint& getWaypoint();
  const Waypoint& getWaypoint() const;

  std::vector<ComponentInfo>& getPointCosts();
  const std::vector<ComponentInfo>& getPointCosts() const;

  std::vector<ComponentInfo>& getPointConstraints();
  const std::vector<ComponentInfo>& getPointConstraints() const;

  std::vector<ComponentInfo>& getPathCosts();
  const std::vector<ComponentInfo>& getPathCosts() const;

  std::vector<ComponentInfo>& getPathConstraints();
  const std::vector<ComponentInfo>& getPathConstraints() const;

  int getType() const;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  bool isComposite() const;

  bool isMove() const;

  void print() const;

private:
  int type_ { static_cast<int>(InstructionType::MOVE_INSTRUCTION) };

  Waypoint waypoint_;

  std::string description_;

  std::vector<ComponentInfo> point_costs_;
  std::vector<ComponentInfo> point_constraints_;

  std::vector<ComponentInfo> path_costs_;
  std::vector<ComponentInfo> path_constraints_;
};

}

#endif // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
