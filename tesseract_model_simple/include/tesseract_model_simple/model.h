/**
 * @file model.h
 * @brief Tesseract Model.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MODEL_SIMPLE_MODEL_H
#define TESSERACT_MODEL_SIMPLE_MODEL_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_model_simple/types.h>
#include <tesseract_model_simple/model_state.h>

namespace tesseract_model_simple
{
class Model
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<Model>;
  using ConstPtr = std::shared_ptr<const Model>;
  using UPtr = std::unique_ptr<Model>;
  using ConstUPtr = std::unique_ptr<const Model>;

  Model() = default;
  virtual ~Model() = default;
  Model(const Model&) = default;
  Model& operator=(const Model&) = default;
  Model(Model&&) = default;
  Model& operator=(Model&&) = default;

  virtual Model::UPtr clone() const = 0;

  virtual void setState(const VariableSets& variable_sets) = 0;
  virtual ModelState::UPtr getState() const = 0;

  virtual ModelState::UPtr getState(const VariableSets& variable_sets) const = 0;

  virtual std::string getName() const = 0;
  virtual std::string getRootLinkName() const = 0;
  virtual std::vector<std::string> getLinkNames() const = 0;
  virtual std::vector<std::string> getActiveLinkNames() const = 0;
  virtual std::vector<std::string> getJointNames() const = 0;
  virtual std::vector<std::string> getActiveJointNames() const = 0;
};

}  // namespace tesseract_model_simple

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
