#ifndef TESSERACT_MODEL_SIMPLE_SCENE_MODEL_STATE_H
#define TESSERACT_MODEL_SIMPLE_SCENE_MODEL_STATE_H

#include <tesseract_model_simple/model_state.h>

namespace tesseract_model_simple
{
class SceneModelState : public ModelState
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<SceneModelState>;
  using ConstPtr = std::shared_ptr<const SceneModelState>;
  using UPtr = std::unique_ptr<SceneModelState>;
  using ConstUPtr = std::unique_ptr<const SceneModelState>;

  void setBaseTransform(const Eigen::Isometry3d& transform) override;
  void applyTransform(const Eigen::Isometry3d& transform) override;
  VariableSets getVariableSets() const override;
  TransformSets getLinkTransformSets() const override;
  TransformSets getJointTransformSets() const override;
  ModelState::UPtr clone() const override;

  std::string name;

  Eigen::Isometry3d base_transform{ Eigen::Isometry3d::Identity() };

  /**  @brief The joint values used for calculating the joint and link transforms */
  VariableSet joints;

  /** @brief The link transforms in world coordinate system */
  tesseract_common::TransformMap link_transforms;

  /** @brief The joint transforms in world coordinate system */
  tesseract_common::TransformMap joint_transforms;
};

}  // namespace tesseract_model_simple

#endif  // TESSERACT_MODEL_SIMPLE_SCENE_MODEL_STATE_H
