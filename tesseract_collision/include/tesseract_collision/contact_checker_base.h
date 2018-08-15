/**
 * @file contact_checker_base.h
 * @brief This is the contact_checker base class
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
#ifndef TESSERACT_COLLISION_CONTACT_CHECKER_BASE_H
#define TESSERACT_COLLISION_CONTACT_CHECKER_BASE_H

#include <tesseract_core/basic_types.h>
#include <tesseract_collision/discrete_contact_manager_base.h>
#include <tesseract_collision/continuous_contact_manager_base.h>
#include <geometric_shapes/shapes.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

namespace tesseract
{
class ContactCheckerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Perform a discrete check using the current state and request
   * @param contacts A list of contact results.
   */
  virtual void calcDistancesDiscrete(ContactResultMap& contacts) = 0;

  /**
   * @brief Calculate distance information for all links in transforms (Discrete Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req        The distance request information.
   * @param transforms The transforms for objects to check collision.
   * @param contacts   A list of contact results.
   */
  virtual void calcDistancesDiscrete(const ContactRequest& req,
                                     const TransformMap& transforms,
                                     ContactResultMap& contacts) const = 0;

  /**
   * @brief Calculate distance information for all links in transforms1/transforms2 (Continuous Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req         The distance request information.
   * @param transforms1 The transforms for every object at start
   * @param transforms2 The transforms for every object at end
   * @param contacts    A list of contact results.
   */
  virtual void calcDistancesContinuous(const ContactRequest& req,
                                       const TransformMap& transforms1,
                                       const TransformMap& transforms2,
                                       ContactResultMap& contacts) const = 0;

  /**
   * @brief Perform a discrete collision check using the current state and request
   * @param contacts A list of contact results.
   */
  virtual void calcCollisionsDiscrete(ContactResultMap& contacts) = 0;

  /**
   * @brief Calculate collision information for all links in transforms (Discrete Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req        The distance request information.
   * @param transforms The transforms for every object
   * @param contacts   A list of contact results.
   */
  virtual void calcCollisionsDiscrete(const ContactRequest& req,
                                      const TransformMap& transforms,
                                      ContactResultMap& contacts) const = 0;

  /**
   * @brief Calculate collision information for all links in transforms1/transfroms2 (Continuous Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req         The distance request information.
   * @param transforms1 The transforms for every object at start
   * @param transforms2 The transforms for every object at end
   * @param contacts    A list of contact results.
   */
  virtual void calcCollisionsContinuous(const ContactRequest& req,
                                        const TransformMap& transforms1,
                                        const TransformMap& transforms2,
                                        ContactResultMap& contacts) const = 0;

  /**
   * @brief Add a object to the checker
   * @param name            The name of the object, must be unique.
   * @param mask_id         User defined id which gets stored in the results structure.
   * @param shapes          A vector of shapes that make up the collision object.
   * @param shape_poses     A vector of poses for each shape, must be same length as shapes
   * @param shape_types     A vector of shape types for encode the collision object. If the vector is of length 1 it is
   * used for all shapes.
   * @param conversion_mode A int identifying a conversion mode for the object. (ex. convert meshes to convex_hulls)
   * @return true if successfully added, otherwise false.
   */
  virtual bool addObject(const std::string& name,
                         const int& mask_id,
                         const std::vector<shapes::ShapeConstPtr>& shapes,
                         const EigenSTL::vector_Affine3d& shape_poses,
                         const CollisionObjectTypeVector& collision_object_types,
                         bool enabled = true) = 0;

  /**
   * @brief Remove an object from the checker
   * @param name The name of the object
   * @return true if successfully removed, otherwise false.
   */
  virtual bool removeObject(const std::string& name) = 0;

  /**
   * @brief Enable an object
   * @param name The name of the object
   */
  virtual void enableObject(const std::string& name) = 0;

  /**
   * @brief Disable an object
   * @param name The name of the object
   */
  virtual void disableObject(const std::string& name) = 0;

  /**
   * @brief Set a collision objects tansforms
   *
   * This is to be used by a collision monitor.
   *
   */
  virtual void setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) = 0;
  virtual void setObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses) = 0;
  virtual void setObjectsTransform(const TransformMap& transforms) = 0;

  /**
   * @brief Set the active contact request information
   * @param req ContactRequest information
   */
  virtual void setContactRequest(const ContactRequest& req) = 0;

  /**
   * @brief Get the active contact request information
   * @return Active contact request information
   */
  virtual const ContactRequest& getContactRequest() const = 0;

  /**
   * @brief Create a standalone discrete contact manager decoupled from parent
   * @param req Contact request information
   * @param transforms
   * @return
   */
  virtual DiscreteContactManagerBasePtr createDiscreteManager(const ContactRequest& req, const TransformMap& transforms) const = 0;

//  ContinuousContactManagerBasePtr createContinuousManager(const ContactRequest& req, const TransformMap& transforms) const;
};
typedef std::shared_ptr<ContactCheckerBase> ContactCheckerBasePtr;
typedef std::shared_ptr<const ContactCheckerBase> ContactCheckerBaseConstPtr;
}
#endif  // TESSERACT_COLLISION_CONTACT_CHECKER_BASE_H
