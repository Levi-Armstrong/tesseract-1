#ifndef TESSERACT_MODEL_SIMPLE_MODEL_STATE_H
#define TESSERACT_MODEL_SIMPLE_MODEL_STATE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_model_simple/types.h>

namespace tesseract_model_simple
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

  virtual void setBaseTransform(const Eigen::Isometry3d& transform) = 0;
  virtual void applyTransform(const Eigen::Isometry3d& transform) = 0;
  virtual VariableSets getVariableSets() const = 0;
  virtual TransformSets getLinkTransformSets() const = 0;
  virtual TransformSets getJointTransformSets() const = 0;
  virtual UPtr clone() const = 0;
};

}  // namespace tesseract_model_simple

#endif  // TESSERACT_MODEL_SIMPLE_MODEL_STATE_H
