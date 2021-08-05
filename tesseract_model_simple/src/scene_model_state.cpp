#include <memory>
#include <tesseract_model_simple/scene_model_state.h>

namespace tesseract_model_simple
{
void SceneModelState::setBaseTransform(const Eigen::Isometry3d& transform)
{
  Eigen::Isometry3d delta = base_transform.inverse() * transform;
  applyTransform(delta);
  base_transform = transform;
}

void SceneModelState::applyTransform(const Eigen::Isometry3d& transform)
{
  for (auto& tf : link_transforms)
    tf.second = tf.second * transform;

  for (auto& tf : joint_transforms)
    tf.second = tf.second * transform;

  base_transform = base_transform * transform;
}

VariableSets SceneModelState::getVariableSets() const { return { std::make_pair(name, joints) }; }

TransformSets SceneModelState::getLinkTransformSets() const { return { std::make_pair(name, link_transforms) }; }

TransformSets SceneModelState::getJointTransformSets() const { return { std::make_pair(name, joint_transforms) }; }

ModelState::UPtr SceneModelState::clone() const
{
  auto state = std::make_unique<SceneModelState>();
  state->name = name;
  state->base_transform = base_transform;
  state->joints = joints;
  state->link_transforms = link_transforms;
  state->joint_transforms = joint_transforms;
  return state;
}
}  // namespace tesseract_model_simple
