#ifndef TESSERACT_COLLISION_COLLISION_MESH_MESH_UNIT_HPP
#define TESSERACT_COLLISION_COLLISION_MESH_MESH_UNIT_HPP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>

namespace tesseract_collision
{
namespace test_suite
{
namespace detail
{
inline void addCollisionObjects(DiscreteContactManager& checker)
{
  ////////////////////////
  // Add sphere to checker
  ////////////////////////
  tesseract_geometry::Mesh::Ptr sphere;

  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  auto faces = std::make_shared<Eigen::VectorXi>();
  int num_faces =
      loadSimplePlyFile(std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.ply", *vertices, *faces, true);
  EXPECT_GT(num_faces, 0);

  sphere = std::make_shared<tesseract_geometry::Mesh>(vertices, faces);
  EXPECT_TRUE(num_faces == sphere->getFaceCount());

  Eigen::Isometry3d sphere_pose;
  sphere_pose.setIdentity();

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(sphere);
  obj1_poses.push_back(sphere_pose);

  checker.addCollisionObject("sphere_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("sphere_link");

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  CollisionShapePtr thin_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses);
  checker.disableCollisionObject("thin_box_link");

  /////////////////////////////////////////////////////////////////
  // Add second sphere to checker. If use_convex_mesh = true
  // then this sphere will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  CollisionShapePtr sphere1 = std::make_shared<tesseract_geometry::Mesh>(vertices, faces);
  Eigen::Isometry3d sphere1_pose;
  sphere1_pose.setIdentity();

  CollisionShapesConst obj3_shapes;
  tesseract_common::VectorIsometry3d obj3_poses;
  obj3_shapes.push_back(sphere1);
  obj3_poses.push_back(sphere1_pose);

  checker.addCollisionObject("sphere1_link", 0, obj3_shapes, obj3_poses);

  /////////////////////////////////////////////
  // Add box and remove
  /////////////////////////////////////////////
  CollisionShapePtr remove_box = std::make_shared<tesseract_geometry::Box>(0.1, 1, 1);
  Eigen::Isometry3d remove_box_pose;
  thin_box_pose.setIdentity();

  CollisionShapesConst obj4_shapes;
  tesseract_common::VectorIsometry3d obj4_poses;
  obj4_shapes.push_back(remove_box);
  obj4_poses.push_back(remove_box_pose);

  checker.addCollisionObject("remove_box_link", 0, obj4_shapes, obj4_poses);
  EXPECT_TRUE(checker.getCollisionObjects().size() == 4);
  EXPECT_TRUE(checker.hasCollisionObject("remove_box_link"));
  checker.removeCollisionObject("remove_box_link");
  EXPECT_FALSE(checker.hasCollisionObject("remove_box_link"));

  /////////////////////////////////////////////
  // Try functions on a link that does not exist
  /////////////////////////////////////////////
  EXPECT_FALSE(checker.removeCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.enableCollisionObject("link_does_not_exist"));
  EXPECT_FALSE(checker.disableCollisionObject("link_does_not_exist"));

  /////////////////////////////////////////////
  // Try to add empty Collision Object
  /////////////////////////////////////////////
  EXPECT_FALSE(
      checker.addCollisionObject("empty_link", 0, CollisionShapesConst(), tesseract_common::VectorIsometry3d()));

  /////////////////////////////////////////////
  // Check sizes
  /////////////////////////////////////////////
  EXPECT_TRUE(checker.getCollisionObjects().size() == 3);
  for (const auto& co : checker.getCollisionObjects())
  {
    EXPECT_TRUE(checker.getCollisionObjectGeometries(co).size() == 1);
    EXPECT_TRUE(checker.getCollisionObjectGeometriesTransforms(co).size() == 1);
    for (const auto& cgt : checker.getCollisionObjectGeometriesTransforms(co))
    {
      EXPECT_TRUE(cgt.isApprox(Eigen::Isometry3d::Identity(), 1e-5));
    }
  }
}
}  // namespace detail

inline void runTest(DiscreteContactManager& checker)
{
  // Add collision objects
  detail::addCollisionObjects(checker);

  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "sphere_link", "sphere1_link" });
  checker.setCollisionMarginData(CollisionMarginData(0));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.0, 1e-5);

  // Test when object is inside another
  tesseract_common::TransformMap location;
  location["sphere_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"] = Eigen::Isometry3d::Identity();
  location["sphere1_link"].translation()(0) = 0.2;
  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  ContactResultMap result;
  checker.contactTest(result, ContactRequest(ContactTestType::ALL));

  ContactResultVector result_vector;
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.size() >= 37);

  ///////////////////////////////////////////////
  // Test object is out side the contact distance
  ///////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();
  checker.setCollisionObjectsTransform(location);

  checker.contactTest(result, ContactRequest(ContactTestType::ALL));
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Edge to Edge)
  /////////////////////////////////////////////////////////////////////////
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  checker.setCollisionMarginData(CollisionMarginData(0.55));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.55, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.52448, 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][0], 0.23776, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][0], 0.76224, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(1, 0, 0)))), 0.00001);

  /////////////////////////////////////////////////////////////////////////////
  // Test object inside the contact distance (Closest Feature Vertex to Vertex)
  /////////////////////////////////////////////////////////////////////////////
  location["sphere1_link"].translation() = Eigen::Vector3d(0, 1, 0);
  result = ContactResultMap();
  result.clear();
  result_vector.clear();

  // The closest feature of the mesh should be edge to edge
  // Use different method for setting transforms
  checker.setCollisionObjectsTransform("sphere1_link", location["sphere1_link"]);
  checker.setCollisionMarginData(CollisionMarginData(0.55));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.55, 1e-5);
  checker.contactTest(result, ContactRequest(ContactTestType::CLOSEST));
  flattenResults(std::move(result), result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.5, 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "sphere_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[0])][1], 0.25, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[static_cast<size_t>(idx[1])][1], 0.75, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);
  EXPECT_GT((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(0, 1, 0)), 0.0);
  EXPECT_LT(std::abs(std::acos((idx[2] * result_vector[0].normal).dot(Eigen::Vector3d(0, 1, 0)))), 0.00001);
}

inline void runTestIssue626(DiscreteContactManager& checker)
{
  ////////////////////////////////////
  // Add collision object 1 to checker
  ////////////////////////////////////
  std::string collision_obj_1_path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/collision_object_1_scaled.ply";
  auto collision_obj_1 = tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(collision_obj_1_path,Eigen::Vector3d(1,1,1), true, true);
  EXPECT_TRUE(collision_obj_1.size() == 1);

  CollisionShapesConst obj1_shapes;
  tesseract_common::VectorIsometry3d obj1_poses;
  obj1_shapes.push_back(collision_obj_1.front());
  obj1_poses.push_back(Eigen::Isometry3d::Identity());

  checker.addCollisionObject("collision_object_1_link", 0, obj1_shapes, obj1_poses, false);
  checker.enableCollisionObject("collision_object_1_link");

  ////////////////////////////////////
  // Add collision object 2 to checker
  ////////////////////////////////////
  std::string collision_obj_2_path = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/collision_object_2_scaled.dae";
  auto collision_obj_2 = tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(collision_obj_2_path,Eigen::Vector3d(1,1,1), true, true);
  EXPECT_TRUE(collision_obj_2.size() == 1);

  CollisionShapesConst obj2_shapes;
  tesseract_common::VectorIsometry3d obj2_poses;
  obj2_shapes.push_back(collision_obj_2.front());
  obj2_poses.push_back(Eigen::Isometry3d::Identity());

  checker.addCollisionObject("collision_object_2_link", 0, obj2_shapes, obj2_poses, false);
  checker.enableCollisionObject("collision_object_2_link");

  ///////////////////////////////////////////////////////////////////
  // Test when object is in collision (Closest Feature Edge to Edge)
  ///////////////////////////////////////////////////////////////////
  checker.setActiveCollisionObjects({ "collision_object_1_link", "collision_object_2_link" });
  checker.setCollisionMarginData(CollisionMarginData(0));
  EXPECT_NEAR(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.0, 1e-5);

  { // Objects should be in collision
    Eigen::Isometry3d collision_obj_1_pose {Eigen::Isometry3d::Identity()};
    collision_obj_1_pose.linear() = Eigen::Quaterniond( -0.364023, -0.1207412, 0.065261, 0.9212219).toRotationMatrix(); // wxyz
    collision_obj_1_pose.translation() = Eigen::Vector3d(0.970507782716257, -0.199022472197488, 0.0057890632254045);

    Eigen::Isometry3d collision_obj_2_pose {Eigen::Isometry3d::Identity()};
    collision_obj_2_pose.translation() = Eigen::Vector3d(1.750, 0.250, -0.020);

    tesseract_common::TransformMap location;
    location["collision_object_1_link"] = collision_obj_1_pose;
    location["collision_object_2_link"] = collision_obj_2_pose;
    checker.setCollisionObjectsTransform(location);

    // Perform collision check
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::ALL));

    ContactResultVector result_vector;
    flattenResults(std::move(result), result_vector);

    EXPECT_TRUE(!result_vector.empty());
  }

  { // Objects should not be in collision
    Eigen::Isometry3d collision_obj_1_pose {Eigen::Isometry3d::Identity()};
    collision_obj_1_pose.linear() = Eigen::Quaterniond( 0.017854, -0.3827401, 0.023661, 0.9233804).toRotationMatrix(); // wxyz
    collision_obj_1_pose.translation() = Eigen::Vector3d(0.979239218615628, -0.0856723302587343, 0.518894463458944);

    tesseract_common::TransformMap location;
    location["collision_object_1_link"] = collision_obj_1_pose;
    checker.setCollisionObjectsTransform(location);

    // Perform collision check
    ContactResultMap result;
    checker.contactTest(result, ContactRequest(ContactTestType::ALL));

    ContactResultVector result_vector;
    flattenResults(std::move(result), result_vector);

    EXPECT_TRUE(result_vector.empty());
  }
}
}  // namespace test_suite
}  // namespace tesseract_collision
#endif  // TESSERACT_COLLISION_COLLISION_MESH_MESH_UNIT_HPP
