#ifndef TESSERACT_MODEL_SIMPLE_COMPOSITE_MODEL_H
#define TESSERACT_MODEL_SIMPLE_COMPOSITE_MODEL_H

#include <tesseract_model_simple/model.h>
#include <tesseract_model_simple/composite_model_state.h>

namespace tesseract_model_simple
{
struct CompositeModelNode
{
  using UPtr = std::unique_ptr<CompositeModelNode>;

  Model* parent;

  std::string parent_link;

  Model* model;

  std::vector<CompositeModelNode::UPtr> children;
};

class CompositeModel : public Model
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<CompositeModel>;
  using ConstPtr = std::shared_ptr<const CompositeModel>;
  using UPtr = std::unique_ptr<CompositeModel>;
  using ConstUPtr = std::unique_ptr<const CompositeModel>;

  CompositeModel(std::string name, Model::UPtr root);
  ~CompositeModel() override = default;
  CompositeModel(const CompositeModel&) = delete;
  CompositeModel& operator=(const CompositeModel&) = delete;
  CompositeModel(CompositeModel&&) = delete;
  CompositeModel& operator=(CompositeModel&&) = delete;

  void addModel(std::string parent, std::string parent_link, Model::UPtr model);

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
  std::string name_;
  CompositeModelState::UPtr current_state_;
  std::unique_ptr<CompositeModelNode> root_;
  std::map<std::string, CompositeModelNode*> nodes_;
  std::map<std::string, Model::UPtr> models_;
};
}  // namespace tesseract_model_simple

#endif  // TESSERACT_MODEL_SIMPLE_COMPOSITE_MODEL_H
