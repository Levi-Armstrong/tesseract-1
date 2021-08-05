#ifndef TESSERACT_MODEL_LINKED_MODEL_STATE_H
#define TESSERACT_MODEL_LINKED_MODEL_STATE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_model_linked/types.h>

namespace tesseract_model_linked
{
class ModelState
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ModelState>;
  using ConstPtr = std::shared_ptr<const ModelState>;
  using UPtr = std::unique_ptr<ModelState>;
  using ConstUPtr = std::unique_ptr<const ModelState>;

  void setBaseTransform(const Eigen::Isometry3d& transform);
  void applyTransform(const Eigen::Isometry3d& transform);
  VariableSets getVariableSets() const;
  TransformSets getLinkTransformSets() const;
  TransformSets getJointTransformSets() const;
  ModelState::UPtr clone() const;

  std::string name;

  Eigen::Isometry3d base_transform{ Eigen::Isometry3d::Identity() };

  /**  @brief The joint values used for calculating the joint and link transforms */
  VariableSet joints;

  /** @brief The link transforms in world coordinate system */
  tesseract_common::TransformMap link_transforms;

  /** @brief The joint transforms in world coordinate system */
  tesseract_common::TransformMap joint_transforms;
};

}  // namespace tesseract_model_linked

#endif  // TESSERACT_MODEL_LINKED_MODEL_STATE_H
