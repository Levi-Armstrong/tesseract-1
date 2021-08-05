#ifndef TESSERACT_MODEL_LINKED_TYPES_H
#define TESSERACT_MODEL_LINKED_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_model_linked
{
using VariableSet = std::map<std::string, double>;
// using VariableSets = std::unordered_map<std::string, VariableSet>;
using TransformSet = tesseract_common::TransformMap;
using TransformSets = tesseract_common::AlignedMap<std::string, TransformSet>;

}  // namespace tesseract_model_linked

#endif  // TESSERACT_MODEL_LINKED_TYPES_H
