#ifndef TESSERACT_MODEL_SIMPLE_TYPES_H
#define TESSERACT_MODEL_SIMPLE_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_model_simple
{
using VariableSet = std::map<std::string, double>;
using VariableSets = std::unordered_map<std::string, VariableSet>;
using TransformSets = tesseract_common::AlignedMap<std::string, tesseract_common::TransformMap>;

}  // namespace tesseract_model_simple

#endif  // TESSERACT_MODEL_SIMPLE_TYPES_H
