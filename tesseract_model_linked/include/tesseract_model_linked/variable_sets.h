#ifndef TESSERACT_MODEL_LINKED_VARIABLE_SETS_H
#define TESSERACT_MODEL_LINKED_VARIABLE_SETS_H

#include <tesseract_model_linked/types.h>

namespace tesseract_model_linked
{
class VariableSets
{
public:
  using Ptr = std::shared_ptr<VariableSets>;
  using ConstPtr = std::shared_ptr<const VariableSets>;

  void assignVariableSet(std::string name, VariableSet variable_set);
  void removeVariableSet(std::string name);
  void updateVariableSet(std::string name, VariableSet variable_set);

  const VariableSet& getVariableSet(const std::string& name) const;
  const std::unordered_map<std::string, VariableSet>& getVariableSets() const;

protected:
  std::unordered_map<std::string, VariableSet> variable_sets_;
};
}  // namespace tesseract_model_linked
#endif  // TESSERACT_MODEL_LINKED_VARIABLE_SETS_H
