#include <tesseract_model_linked/variable_sets.h>

namespace tesseract_model_linked
{
void VariableSets::assignVariableSet(std::string name, VariableSet variable_set)
{
  variable_sets_[name] = variable_set;
}

void VariableSets::removeVariableSet(std::string name) { variable_sets_.erase(name); }

void VariableSets::updateVariableSet(std::string name, VariableSet variable_set)
{
  VariableSet& vs = variable_sets_[name];
  vs.merge(variable_set);
}

const VariableSet& VariableSets::getVariableSet(const std::string& name) const { return variable_sets_.at(name); }

const std::unordered_map<std::string, VariableSet>& VariableSets::getVariableSets() const { return variable_sets_; }
}  // namespace tesseract_model_linked
