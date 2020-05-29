#include <tesseract_command_language/move_instruction.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(Waypoint waypoint) : waypoint_(std::move(waypoint)) {}

void MoveInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
const Waypoint& MoveInstruction::getWaypoint() const { return waypoint_; }

/** @todo Not sure if this belogs here or in the waypoint? */
//void MoveInstruction::setTCP(Eigen::Isometry3d tcp) { tcp_ = tcp; }
//const Eigen::Isometry3d& MoveInstruction::getTCP() const { return tcp_; }

void MoveInstruction::addCost(ComponentInfo component) { costs_.push_back(component); }
const std::vector<ComponentInfo>& MoveInstruction::getCosts() const { return costs_; }

void MoveInstruction::addConstraint(ComponentInfo component) { constraints_.push_back(component); }
const std::vector<ComponentInfo>& MoveInstruction::getConstraints() const { return constraints_; }

void MoveInstruction::addPathCost(ComponentInfo component) { path_costs_.push_back(component); }
const std::vector<ComponentInfo>& MoveInstruction::getPathCosts() const { return path_costs_; }

void MoveInstruction::addPathConstraint(ComponentInfo component) { path_constraints_.push_back(component); }
const std::vector<ComponentInfo>& MoveInstruction::getPathConstraints() const { return path_constraints_; }

int MoveInstruction::getType() const { return type_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

bool MoveInstruction::isComposite() const { return false; }

bool MoveInstruction::isMove() const { return true; }

void MoveInstruction::print() const { }

bool MoveInstruction::isLinearMove() const { return (move_type_ == MoveInstructionType::LINEAR); }

bool MoveInstruction::isFreespaceMove() const { return (move_type_ == MoveInstructionType::FREESPACE); }

bool MoveInstruction::isCircularMove() const { return (move_type_ == MoveInstructionType::CIRCULAR); }
}
