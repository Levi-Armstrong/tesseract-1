/**
 * @file rep_inverse_kinematics.h
 * @brief Robot with External Positioner Inverse kinematics functions.
 *
 * @author Levi Armstrong
 * @date June 25, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_REP_INVERSE_KINEMATICS_H
#define TESSERACT_KINEMATICS_REP_INVERSE_KINEMATICS_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/types.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::RobotWithExternalPositionerInvKin)
#endif  // SWIG

namespace tesseract_kinematics
{
/**
 * @brief Robot With External Positioner Inverse kinematic implementation.
 *
 * In this kinematic arrangement the base link is the tip link of the external positioner and the tip link is the
 * tip link of the manipulator. Therefore all provided target poses are expected to the tip link of the positioner.
 */
class RobotWithExternalPositionerInvKin : public InverseKinematics
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<RobotWithExternalPositionerInvKin>;
  using ConstPtr = std::shared_ptr<const RobotWithExternalPositionerInvKin>;
  using UPtr = std::unique_ptr<RobotWithExternalPositionerInvKin>;
  using ConstUPtr = std::unique_ptr<const RobotWithExternalPositionerInvKin>;

  RobotWithExternalPositionerInvKin() = default;
  ~RobotWithExternalPositionerInvKin() override = default;
  RobotWithExternalPositionerInvKin(const RobotWithExternalPositionerInvKin&) = delete;
  RobotWithExternalPositionerInvKin& operator=(const RobotWithExternalPositionerInvKin&) = delete;
  RobotWithExternalPositionerInvKin(RobotWithExternalPositionerInvKin&&) = delete;
  RobotWithExternalPositionerInvKin& operator=(RobotWithExternalPositionerInvKin&&) = delete;

  //  bool update() override;

  void synchronize(const std::vector<std::string>& joint_names) override;

  IKSolutions calcInvKin(const Eigen::Isometry3d& pose,
                         const std::string& link_name,
                         const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  std::vector<std::string> getJointNames() const override;
  Eigen::Index numJoints() const override;
  std::string getBaseLinkName() const override;
  std::vector<std::string> getWorkingFrames() const final;
  std::vector<std::string> getTipLinkName() const override;
  std::string getName() const override;
  std::string getSolverName() const override;
  InverseKinematics::UPtr clone() const override;

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            const tesseract_common::TransformMap& current_transforms,
            std::string name);

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            std::string name,
            std::string solver_name = "RobotWithExternalPositionerInvKin");

  /**
   * @brief Initializes Inverse Kinematics for a robot on a positioner
   * @param scene_graph The Tesseract Scene Graph
   * @param manipulator
   * @param manipulator_reach
   * @param positioner
   * @param positioner_sample_resolution
   * @param robot_to_positioner
   * @param name The name of the kinematic object
   * @param solver_name The name given to the solver. This is exposed so you may have same solver with different
   * sampling resolutions
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            InverseKinematics::Ptr manipulator,
            double manipulator_reach,
            ForwardKinematics::Ptr positioner,
            Eigen::VectorXd positioner_sample_resolution,
            const Eigen::Isometry3d& robot_to_positioner,
            std::string name,
            std::string solver_name = "RobotWithExternalPositionerInvKin");

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

private:
  bool initialized_{ false };                               /**< @brief Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_; /**< @brief Tesseract Scene Graph */
  ForwardKinematics::ConstPtr sync_fwd_kin_;                /**< @brief Synchronized forward kinematics object */
  std::vector<Eigen::Index> sync_joint_map_;                /**< @brief Synchronized joint solution remapping */
  InverseKinematics::Ptr manip_inv_kin_;
  double manip_reach_{ 0 };
  ForwardKinematics::Ptr positioner_fwd_kin_;
  Eigen::VectorXd positioner_sample_resolution_;
  Eigen::Isometry3d manip_base_to_positioner_base_;
  unsigned dof_;
  SynchronizableData data_;      /**< @brief The current data that may be synchronized */
  SynchronizableData orig_data_; /**< @brief The data prior to synchronization */
  std::vector<Eigen::VectorXd> dof_range_;
  std::string name_;                                               /**< @brief Name of the kinematic chain */
  std::string solver_name_{ "RobotWithExternalPositionerInvKin" }; /**< @brief Name of this solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const RobotWithExternalPositionerInvKin& kin);

  /** @brief calcFwdKin helper function */
  IKSolutions calcInvKinHelper(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void nested_ik(IKSolutions& solutions,
                 int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& target_pose,
                 Eigen::VectorXd& positioner_pose,
                 const Eigen::Ref<const Eigen::VectorXd>& seed) const;

  void ikAt(IKSolutions& solutions,
            const Eigen::Isometry3d& target_pose,
            Eigen::VectorXd& positioner_pose,
            const Eigen::Ref<const Eigen::VectorXd>& seed) const;
};
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_REP_INVERSE_KINEMATICS_H
