#ifndef TESSERACT_MODEL_LINKED_MODEL_H
#define TESSERACT_MODEL_LINKED_MODEL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_model_linked/model_state_solver.h>

namespace tesseract_model_linked
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

  using Parent = std::pair<Model*, std::string>;
  using Child = std::pair<Model::UPtr, std::string>;
  using Children = std::vector<Child>;

  virtual ~Model() = default;
  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;
  Model(Model&&) = delete;
  Model& operator=(Model&&) = delete;

  Model(tesseract_scene_graph::SceneGraph::UPtr scene_graph);

  void setParent(Model* parent_model, std::string parent_link_name);
  Parent getParent() const;

  void addChild(Model::UPtr child_model, std::string attach_link_name);
  Children& getChildren();
  const Children& getChildren() const;

  Model::UPtr clone() const;

  void setState(const VariableSets& variable_sets);
  ModelState::UPtr getState() const;

  ModelState::UPtr getState(const VariableSets& variable_sets) const;

  std::string getName() const;
  std::string getRootLinkName() const;

  std::vector<std::string> getLinkNames(bool recursive = false) const;
  std::vector<std::string> getActiveLinkNames(bool recursive = false) const;
  std::vector<std::string> getJointNames(bool recursive = false) const;
  std::vector<std::string> getActiveJointNames(bool recursive = false) const;

protected:
  tesseract_scene_graph::SceneGraph::UPtr scene_graph_;
  Model* parent_model_;
  std::string parent_link_name_;
  Children children_;

  ModelStateSolver::UPtr state_solver_;
  ModelState::UPtr current_state_;
  std::vector<std::string> link_names_;
  std::vector<std::string> active_link_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> active_joint_names_;
};
}  // namespace tesseract_model_linked
#endif  // TESSERACT_MODEL_LINKED_MODEL_H
