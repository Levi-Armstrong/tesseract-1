/**
 * @file state_solver.h
 * @brief Tesseract Environment State Solver Interface.
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
#ifndef TESSERACT_MODEL_LINKED_MODEL_STATE_SOLVER_H
#define TESSERACT_MODEL_LINKED_MODEL_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <memory>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_model_linked/types.h>
#include <tesseract_model_linked/model_state.h>
#include <tesseract_model_linked/model_state_set.h>

#ifdef SWIG
%shared_ptr(tesseract_model_simple::StateSolver)
#endif  // SWIG

namespace tesseract_model_linked
{
class ModelStateSolver
{
public:
  using Ptr = std::shared_ptr<ModelStateSolver>;
  using ConstPtr = std::shared_ptr<const ModelStateSolver>;
  using UPtr = std::unique_ptr<ModelStateSolver>;
  using ConstUPtr = std::unique_ptr<const ModelStateSolver>;

  ModelStateSolver() = default;
  virtual ~ModelStateSolver() = default;
  ModelStateSolver(const ModelStateSolver&) = delete;
  ModelStateSolver& operator=(const ModelStateSolver&) = delete;
  ModelStateSolver(ModelStateSolver&&) = delete;
  ModelStateSolver& operator=(ModelStateSolver&&) = delete;

  virtual bool init(const tesseract_scene_graph::SceneGraph& scene_graph, ModelState* state);

  virtual void setState(const VariableSet& variable_set);
  virtual ModelState::UPtr getState() const;

  virtual ModelState::UPtr getState(const VariableSet& variable_set) const;

  virtual UPtr clone() const;

protected:
  std::string name_;
  std::string root_name_;
  ModelState* current_state_;
  KDL::Tree kdl_tree_;                                         /**< KDL tree object */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */

  void calculateTransforms(ModelState& state,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(ModelState& state,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  bool createKDETree(const tesseract_scene_graph::SceneGraph& scene_graph);
};
}  // namespace tesseract_model_linked

#endif  // TESSERACT_MODEL_LINKED_MODEL_STATE_SOLVER_H
