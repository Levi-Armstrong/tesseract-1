#ifndef TESSERACT_MODEL_SIMPLE_COMPOSITE_MODEL_STATE_H
#define TESSERACT_MODEL_SIMPLE_COMPOSITE_MODEL_STATE_H

#include <tesseract_model_simple/model_state.h>

namespace tesseract_model_simple
{
class CompositeModelState : public ModelState
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<CompositeModelState>;
  using ConstPtr = std::shared_ptr<const CompositeModelState>;
  using UPtr = std::unique_ptr<CompositeModelState>;
  using ConstUPtr = std::unique_ptr<const CompositeModelState>;

  void addModelState(ModelState::UPtr model_state);

  void setBaseTransform(const Eigen::Isometry3d& transform) override;
  void applyTransform(const Eigen::Isometry3d& transform) override;
  VariableSets getVariableSets() const override;
  TransformSets getLinkTransformSets() const override;
  TransformSets getJointTransformSets() const override;
  ModelState::UPtr clone() const override;

  Eigen::Isometry3d base_transform{ Eigen::Isometry3d::Identity() };
  std::vector<ModelState::UPtr> model_states;
};
}  // namespace tesseract_model_simple

#endif  // TESSERACT_MODEL_SIMPLE_COMPOSITE_MODEL_STATE_H
