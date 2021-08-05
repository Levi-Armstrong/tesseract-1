#ifndef TESSERACT_MODEL_LINKED_MODEL_STATE_SET_H
#define TESSERACT_MODEL_LINKED_MODEL_STATE_SET_H

#include <tesseract_model_linked/model_state.h>

namespace tesseract_model_linked
{
class ModelStateSet
{
public:
  using Ptr = std::shared_ptr<ModelState>;
  using ConstPtr = std::shared_ptr<const ModelState>;

  void assign(std::string name, ModelState::UPtr model_state);
  void remove(std::string name);

  void updateVariableSet(std::string name, const VariableSet& variable_set);
  void updateLinkTransformSet(std::string name, const TransformSet& transform_set);
  void updateJointTransformSet(std::string name, const TransformSet& transform_set);

  const ModelState& getModelState(const std::string& name) const;
  const std::unordered_map<std::string, ModelState::UPtr>& getModelStateSet() const;

protected:
  std::unordered_map<std::string, ModelState::UPtr> model_state_set_;
};

}  // namespace tesseract_model_linked
#endif  // TESSERACT_MODEL_LINKED_MODEL_STATE_SET_H
