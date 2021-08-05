#include <tesseract_model_simple/composite_model_state.h>

namespace tesseract_model_simple
{
void CompositeModelState::addModelState(ModelState::UPtr model_state)
{
  model_states.push_back(std::move(model_state));
}
void CompositeModelState::setBaseTransform(const Eigen::Isometry3d& transform)
{
  Eigen::Isometry3d delta = base_transform.inverse() * transform;
  applyTransform(delta);
  base_transform = transform;
}

void CompositeModelState::applyTransform(const Eigen::Isometry3d& transform)
{
  for (auto& ms : model_states)
    ms->applyTransform(transform);

  base_transform = base_transform * transform;
}

VariableSets CompositeModelState::getVariableSets() const
{
  VariableSets vs;
  for (const auto& ms : model_states)
    vs.merge(ms->getVariableSets());

  return vs;
}

TransformSets CompositeModelState::getLinkTransformSets() const
{
  TransformSets ts;
  for (const auto& ms : model_states)
    ts.merge(ms->getLinkTransformSets());

  return ts;
}

TransformSets CompositeModelState::getJointTransformSets() const
{
  TransformSets ts;
  for (const auto& ms : model_states)
    ts.merge(ms->getJointTransformSets());
  return ts;
}

ModelState::UPtr CompositeModelState::clone() const
{
  auto cms = std::make_unique<CompositeModelState>();
  cms->base_transform = base_transform;
  cms->model_states.reserve(model_states.size());
  for (const auto& ms : model_states)
    cms->model_states.push_back(ms->clone());

  return cms;
}
}  // namespace tesseract_model_simple
