#ifndef TESSERACT_MODEL_SIMPLE_SCENE_MODEL_H
#define TESSERACT_MODEL_SIMPLE_SCENE_MODEL_H

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_model_simple/model.h>
#include <tesseract_model_simple/state_solver.h>

namespace tesseract_model_simple
{
class SceneModel : public Model
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<SceneModel>;
  using ConstPtr = std::shared_ptr<const SceneModel>;
  using UPtr = std::unique_ptr<SceneModel>;
  using ConstUPtr = std::unique_ptr<const SceneModel>;

  virtual ~SceneModel() override = default;
  SceneModel(const SceneModel&) = delete;
  SceneModel& operator=(const SceneModel&) = delete;
  SceneModel(SceneModel&&) = delete;
  SceneModel& operator=(SceneModel&&) = delete;

  SceneModel(tesseract_scene_graph::SceneGraph::UPtr scene_graph);

  Model::UPtr clone() const override;

  void setState(const VariableSets& variable_sets) override;
  ModelState::UPtr getState() const override;

  ModelState::UPtr getState(const VariableSets& variable_sets) const override;

  std::string getName() const override;
  std::string getRootLinkName() const override;
  std::vector<std::string> getLinkNames() const override;
  std::vector<std::string> getActiveLinkNames() const override;
  std::vector<std::string> getJointNames() const override;
  std::vector<std::string> getActiveJointNames() const override;

protected:
  tesseract_scene_graph::SceneGraph::UPtr scene_graph_;
  StateSolver::UPtr state_solver_;
  ModelState::UPtr current_state_;
  std::vector<std::string> link_names_;
  std::vector<std::string> active_link_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> active_joint_names_;
};
}  // namespace tesseract_model_simple
#endif  // TESSERACT_MODEL_SIMPLE_SCENE_MODEL_H
