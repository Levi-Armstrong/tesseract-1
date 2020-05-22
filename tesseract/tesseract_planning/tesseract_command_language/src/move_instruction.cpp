#include <tesseract_command_language/move_instruction.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(Waypoint waypoint) : waypoint_(std::move(waypoint)) {}

Waypoint& MoveInstruction::getWaypoint() { return waypoint_; }
const Waypoint& MoveInstruction::getWaypoint() const { return waypoint_; }

std::vector<ComponentInfo>& MoveInstruction::getPointCosts() { return point_costs_; }
const std::vector<ComponentInfo>& MoveInstruction::getPointCosts() const { return point_costs_; }

std::vector<ComponentInfo>& MoveInstruction::getPointConstraints() { return point_constraints_; }
const std::vector<ComponentInfo>& MoveInstruction::getPointConstraints() const { return point_constraints_; }

std::vector<ComponentInfo>& MoveInstruction::getPathCosts() { return path_costs_; }
const std::vector<ComponentInfo>& MoveInstruction::getPathCosts() const { return path_costs_; }

std::vector<ComponentInfo>& MoveInstruction::getPathConstraints() { return path_constraints_; }
const std::vector<ComponentInfo>& MoveInstruction::getPathConstraints() const { return path_constraints_; }

int MoveInstruction::getType() const { return type_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

bool MoveInstruction::isComposite() const { return false; }

bool MoveInstruction::isMove() const { return true; }

void MoveInstruction::print() const { }
}
