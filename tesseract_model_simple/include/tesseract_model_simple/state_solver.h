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
#ifndef TESSERACT_MODEL_SIMPLE_STATE_SOLVER_H
#define TESSERACT_MODEL_SIMPLE_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <memory>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_model_simple/types.h>
#include <tesseract_model_simple/scene_model_state.h>

#ifdef SWIG
%shared_ptr(tesseract_model_simple::StateSolver)
#endif  // SWIG

namespace tesseract_model_simple
{
class StateSolver
{
public:
  using Ptr = std::shared_ptr<StateSolver>;
  using ConstPtr = std::shared_ptr<const StateSolver>;
  using UPtr = std::unique_ptr<StateSolver>;
  using ConstUPtr = std::unique_ptr<const StateSolver>;

  StateSolver() = default;
  virtual ~StateSolver() = default;
  StateSolver(const StateSolver&) = delete;
  StateSolver& operator=(const StateSolver&) = delete;
  StateSolver(StateSolver&&) = delete;
  StateSolver& operator=(StateSolver&&) = delete;

  virtual bool init(const tesseract_scene_graph::SceneGraph& scene_graph);

  virtual void setState(const VariableSet& variable_set);
  virtual ModelState::UPtr getState() const;

  virtual ModelState::UPtr getState(const VariableSet& variable_set) const;

  virtual UPtr clone() const;

protected:
  std::string root_name_;
  SceneModelState::UPtr current_state_;
  KDL::Tree kdl_tree_;                                         /**< KDL tree object */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */

  void calculateTransforms(SceneModelState& state,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(SceneModelState& state,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  bool createKDETree(const tesseract_scene_graph::SceneGraph& scene_graph);
};
}  // namespace tesseract_model_simple

#endif  // TESSERACT_MODEL_SIMPLE_STATE_SOLVER_H
