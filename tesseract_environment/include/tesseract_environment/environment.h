/**
 * @file Environment.h
 * @brief Tesseract Environment.
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
#ifndef TESSERACT_ENVIRONMENT_ENVIRONMENT_H
#define TESSERACT_ENVIRONMENT_ENVIRONMENT_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <shared_mutex>
#include <chrono>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/commands.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager_factory.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

#ifdef SWIG
%shared_ptr(tesseract_environment::Environment)
#endif  // SWIG

namespace tesseract_environment
{
/**
 * @brief Function signature for adding additional callbacks for looking up TCP information
 *
 * The function should throw and exception if not located
 */
using FindTCPCallbackFn = std::function<Eigen::Isometry3d(const tesseract_common::ManipulatorInfo&)>;

class Environment
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<Environment>;
  using ConstPtr = std::shared_ptr<const Environment>;
  using UPtr = std::unique_ptr<Environment>;
  using ConstUPtr = std::unique_ptr<const Environment>;

  /**
   * @brief Default constructor
   * @param register_default_contant_managers Indicate if the default contact managers should be registered
   */
  Environment(bool register_default_contact_managers = true);

  virtual ~Environment() = default;
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
  Environment(Environment&&) = delete;
  Environment& operator=(Environment&&) = delete;

  /**
   * @brief Initialize the Environment
   *
   * The templated class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const Commands& commands);

  /**
   * @brief Initialize the Environment
   *
   * The templated class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_srdf::SRDFModel::ConstPtr& srdf_model = nullptr);

  bool init(const std::string& urdf_string, const tesseract_common::ResourceLocator::ConstPtr& locator);

  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const tesseract_common::ResourceLocator::ConstPtr& locator);

  bool init(const tesseract_common::fs::path& urdf_path, const tesseract_common::ResourceLocator::ConstPtr& locator);

  bool init(const tesseract_common::fs::path& urdf_path,
            const tesseract_common::fs::path& srdf_path,
            const tesseract_common::ResourceLocator::ConstPtr& locator);

  /**
   * @brief Clone the environment
   * @return A clone of the environment
   */
  Environment::Ptr clone() const;

  /**
   * @brief reset to initialized state
   * @details If the environment has not been initialized then this returns false
   * @return True if environment was successfully reset, otherwise false.
   */
  bool reset();

  /** @brief clear content and uninitialize */
  void clear();

  /** @brief check if the environment is initialized */
  bool isInitialized() const;

  /**
   * @brief Get the current revision number
   * @return Revision number
   */
  int getRevision() const;

  /**
   * @brief Get Environment command history post initialization
   * @return List of commands
   */
  Commands getCommandHistory() const;

  /**
   * @brief Applies the commands to the environment
   * @param commands Commands to be applied to the environment
   * @return true if successful. If returned false, then only a partial set of commands have been applied. Call
   * getCommandHistory to check. Some commands are not checked for success
   */
  bool applyCommands(const Commands& commands);

  /**
   * @brief Apply command to the environment
   * @param command Command to be applied to the environment
   * @return true if successful. If returned false, then the command have not been applied.
   * Some type of Command are not checked for success
   */
  bool applyCommand(Command::ConstPtr command);

  /**
   * @brief Get the Scene Graph
   * @return SceneGraphConstPtr
   */
  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;

  /**
   * @brief Get a joint group by name
   * @param group_name The group name
   * @return A joint group
   */
  tesseract_kinematics::JointGroup::UPtr getJointGroup(const std::string& group_name) const;

  /**
   * @brief Get a joint group given a vector of joint names
   * @param name The name to assign to the joint group
   * @param joint_names The joint names that make up the group
   * @return A joint group
   */
  tesseract_kinematics::JointGroup::UPtr getJointGroup(const std::string& name,
                                                       const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a kinematics group given group name and solver name
   * @details If ik_solver_name is empty it will choose the first ik solver for the group
   * @param group_name The group name
   * @param ik_solver_name The IK solver name
   * @return A kinematics group
   */
  tesseract_kinematics::KinematicGroup::UPtr getKinematicsGroup(const std::string& group_name,
                                                                std::string ik_solver_name = "") const;

  /**
   * @brief Find tool center point provided in the manipulator info
   *
   * If manipulator information tcp is defined as a string it does the following
   *    - First check if manipulator info is empty, if so return identity
   *    - Next if not empty, it checks if the manipulator manager has tcp defined for the manipulator group
   *    - Next if not found, it looks up the tcp name in the EnvState along with manipulator tip link to calculate tcp
   *    - Next if not found, it leverages the user defind callbacks to try an locate the tcp information.
   *    - Next throw an exception, because no tcp information was located.
   *
   * @param manip_info The manipulator info
   * @return The tool center point
   */
  Eigen::Isometry3d findTCP(const tesseract_common::ManipulatorInfo& manip_info) const;

  /**
   * @brief This allows for user defined callbacks for looking up TCP information
   * @param fn User defind callback function for locating TCP information
   */
  void addFindTCPCallback(FindTCPCallbackFn fn);

  /**
   * @brief This get the current find tcp callbacks stored in the environment
   * @return A vector of callback functions
   */
  std::vector<FindTCPCallbackFn> getFindTCPCallbacks() const;

  /**
   * @brief Set resource locator for environment
   * @param locator The resource locator
   */
  void setResourceLocator(tesseract_common::ResourceLocator::ConstPtr locator);

  /**
   * @brief Get the resource locator assigned
   * @details This can be a nullptr
   * @return The resource locator assigned to the environment
   */
  tesseract_common::ResourceLocator::ConstPtr getResourceLocator() const;

  /** @brief Give the environment a name */
  void setName(const std::string& name);

  /** @brief Get the name of the environment
   *
   * This may be empty, if so check urdf name
   */
  const std::string& getName() const;

  /**
   * @brief Set the current state of the environment
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  void setState(const std::unordered_map<std::string, double>& joints);
  void setState(const std::vector<std::string>& joint_names, const Eigen::Ref<const Eigen::VectorXd>& joint_values);

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  tesseract_scene_graph::SceneState getState(const std::unordered_map<std::string, double>& joints) const;
  tesseract_scene_graph::SceneState getState(const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** @brief Get the current state of the environment */
  tesseract_scene_graph::SceneState::ConstPtr getCurrentState() const;

  /** @brief Get the current state timestamp */
  std::chrono::high_resolution_clock::duration getCurrentStateTimestamp() const;

  /**
   * @brief Get a link in the environment
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  tesseract_scene_graph::Link::ConstPtr getLink(const std::string& name) const;

  /**
   * @brief Get joint by name
   * @param name The name of the joint
   * @return Joint Const Pointer
   */
  tesseract_scene_graph::Joint::ConstPtr getJoint(const std::string& name) const;

  /**
   * @brief Gets the limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @return The joint limits set for the given joint
   */
  tesseract_scene_graph::JointLimits::ConstPtr getJointLimits(const std::string& joint_name) const;

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be condisdered during collision checking, otherwise false
   */
  bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Get a given links visibility setting
   * @return True if should be visible, otherwise false
   */
  bool getLinkVisibility(const std::string& name) const;

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixConstPtr
   */
  tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  std::vector<std::string> getJointNames() const;

  /**
   * @brief Get a vector of active joint names in the environment
   * @return A vector of active joint names
   */
  std::vector<std::string> getActiveJointNames() const;

  /**
   * @brief Get the current state of the environment
   *
   * Order should be the same as getActiveJointNames()
   *
   * @return A vector of joint values
   */
  Eigen::VectorXd getCurrentJointValues() const;

  /**
   * @brief Get the current joint values for a vector of joints
   *
   * Order should be the same as the input vector
   *
   * @return A vector of joint values
   */
  Eigen::VectorXd getCurrentJointValues(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the root link name
   * @return String
   */
  std::string getRootLinkName() const;

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  std::vector<std::string> getLinkNames() const;

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames() const;

  /**
   * @brief Get a vector of active link names affected by the provided joints in the environment
   * @param joint_names A list of joint names
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a vector of static link names in the environment
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames() const;

  /**
   * @brief Get a vector of static link names not affected by the provided joints in the environment
   * @param joint_names A list of joint names
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get all of the links transforms
   *
   * Order should be the same as getLinkNames()
   *
   * @return Get a vector of transforms for all links in the environment.
   */
  tesseract_common::VectorIsometry3d getLinkTransforms() const;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  Eigen::Isometry3d getLinkTransform(const std::string& link_name) const;

  /**
   * @brief Returns a clone of the environments state solver
   *
   * The Environment::getState contains mutex's which is may not be needed in all motion planners. This allows the user
   * to get snap shot of the environment to calculate the state.
   *
   * @return A clone of the environments state solver
   */
  tesseract_scene_graph::StateSolver::UPtr getStateSolver() const;

  /**
   * @brief Set the active discrete contact manager
   * @param name The name used to registar the contact manager
   * @return True of name exists in DiscreteContactManagerFactory
   */
  bool setActiveDiscreteContactManager(const std::string& name);

  /** @brief Get a copy of the environments active discrete contact manager */
  tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager() const;

  /** @brief Get a copy of the environments available discrete contact manager by name */
  tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager(const std::string& name) const;

  /**
   * @brief Set the active continuous contact manager
   * @param name The name used to registar the contact manager
   * @return True of name exists in ContinuousContactManagerFactory
   */
  bool setActiveContinuousContactManager(const std::string& name);

  /** @brief Get a copy of the environments active continuous contact manager */
  tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager() const;

  /** @brief Get a copy of the environments available continuous contact manager by name */
  tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager(const std::string& name) const;

  /** @brief Get the environment collision margin data */
  tesseract_common::CollisionMarginData getCollisionMarginData() const;

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  bool registerDiscreteContactManager(const std::string& name,
                                      tesseract_collision::DiscreteContactManagerFactoryCreateMethod create_function);

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  bool
  registerContinuousContactManager(const std::string& name,
                                   tesseract_collision::ContinuousContactManagerFactoryCreateMethod create_function);

  /**
   * @brief Register Default Contact Managers
   * @return True if successful, otherwis false
   */
  bool registerDefaultContactManagers();

protected:
  /** @brief Identifies if the object has been initialized */
  bool initialized_{ false };

  /** @brief This increments when the scene graph is modified */
  int revision_{ 0 };

  /** @brief This is the revision number after initialization used when reset is called */
  int init_revision_{ 0 };

  /** @brief The history of commands applied to the environment after intialization */
  Commands commands_;

  /** @brief Tesseract Scene Graph */
  tesseract_scene_graph::SceneGraph::Ptr scene_graph_;

  /** @brief Tesseract Scene Graph Const */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_const_;

  /** @brief The kinematics information */
  tesseract_srdf::KinematicsInformation kinematics_information_;

  /** @brief The kinematics factory */
  tesseract_kinematics::KinematicsPluginFactory kinematics_factory_;

  /** @brief Current state of the environment */
  tesseract_scene_graph::SceneState::Ptr current_state_;

  /** @brief Current state timestamp */
  std::chrono::high_resolution_clock::duration current_state_timestamp_;

  /** @brief Tesseract State Solver */
  tesseract_scene_graph::MutableStateSolver::UPtr state_solver_;

  /** @brief The function used to determine if two objects are allowed in collision */
  tesseract_collision::IsContactAllowedFn is_contact_allowed_fn_;

  /** @brief A vector of user defined callbacks for locating tool center point */
  std::vector<FindTCPCallbackFn> find_tcp_cb_;

  /** @brief This indicates that the default collision checker 'Bullet' should be registered */
  bool register_default_contact_managers_{ true };

  /** @brief Used when initialized by URDF_STRING, URDF_STRING_SRDF_STRING, URDF_PATH, and URDF_PATH_SRDF_PATH */
  tesseract_common::ResourceLocator::ConstPtr resource_locator_;

  /** @brief The collision margin data */
  tesseract_collision::CollisionMarginData collision_margin_data_;

  /** @brief The discrete contact manager object */
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager_;

  /** @brief The continuous contact manager object */
  tesseract_collision::ContinuousContactManager::Ptr continuous_manager_;

  /** @brief Name of active descrete contact manager */
  std::string discrete_manager_name_;

  /** @brief Name of active continuous contact manager */
  std::string continuous_manager_name_;

  /** @brief Descrete contact manager factory */
  tesseract_collision::DiscreteContactManagerFactory discrete_factory_;

  /** @brief Continuous contact manager factory */
  tesseract_collision::ContinuousContactManagerFactory continuous_factory_;

  /** @brief The environment can be accessed from multiple threads, need use mutex throughout */
  mutable std::shared_mutex mutex_;

  /** This will update the contact managers transforms */
  void currentStateChanged();

  /** This will notify the state solver that the environment has changed */
  void environmentChanged();

private:
  bool removeLinkHelper(const std::string& name);

  void getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                          tesseract_common::VectorIsometry3d& shape_poses,
                          const tesseract_scene_graph::Link& link) const;

  bool registerDefaultContactManagersHelper();
  bool setActiveDiscreteContactManagerHelper(const std::string& name);
  bool setActiveContinuousContactManagerHelper(const std::string& name);

  tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManagerHelper(const std::string& name) const;

  tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManagerHelper(const std::string& name) const;

  bool initHelper(const Commands& commands);
  Commands getInitCommands(const tesseract_scene_graph::SceneGraph& scene_graph,
                           const tesseract_srdf::SRDFModel::ConstPtr& srdf_model = nullptr) const;

  /** @brief Apply Command Helper which does not lock */
  bool applyCommandsHelper(const Commands& commands);

  // Command Helper function
  bool applyAddCommand(AddLinkCommand::ConstPtr cmd);
  bool applyMoveLinkCommand(const MoveLinkCommand::ConstPtr& cmd);
  bool applyMoveJointCommand(const MoveJointCommand::ConstPtr& cmd);
  bool applyRemoveLinkCommand(const RemoveLinkCommand::ConstPtr& cmd);
  bool applyRemoveJointCommand(const RemoveJointCommand::ConstPtr& cmd);
  bool applyReplaceJointCommand(const ReplaceJointCommand::ConstPtr& cmd);
  bool applyChangeLinkOriginCommand(const ChangeLinkOriginCommand::ConstPtr& cmd);
  bool applyChangeJointOriginCommand(const ChangeJointOriginCommand::ConstPtr& cmd);
  bool applyChangeLinkCollisionEnabledCommand(const ChangeLinkCollisionEnabledCommand::ConstPtr& cmd);
  bool applyChangeLinkVisibilityCommand(const ChangeLinkVisibilityCommand::ConstPtr& cmd);
  bool applyAddAllowedCollisionCommand(const AddAllowedCollisionCommand::ConstPtr& cmd);
  bool applyRemoveAllowedCollisionCommand(const RemoveAllowedCollisionCommand::ConstPtr& cmd);
  bool applyRemoveAllowedCollisionLinkCommand(const RemoveAllowedCollisionLinkCommand::ConstPtr& cmd);
  bool applyAddSceneGraphCommand(AddSceneGraphCommand::ConstPtr cmd);
  bool applyChangeJointPositionLimitsCommand(const ChangeJointPositionLimitsCommand::ConstPtr& cmd);
  bool applyChangeJointVelocityLimitsCommand(const ChangeJointVelocityLimitsCommand::ConstPtr& cmd);
  bool applyChangeJointAccelerationLimitsCommand(const ChangeJointAccelerationLimitsCommand::ConstPtr& cmd);
  bool applyAddKinematicsInformationCommand(const AddKinematicsInformationCommand::ConstPtr& cmd);
  bool applyChangeCollisionMarginsCommand(const ChangeCollisionMarginsCommand::ConstPtr& cmd);
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
