
#include "tesseract_collision/bullet/bullet_discrete_managers.h"
#include "tesseract_collision/bullet/bullet_cast_managers.h"
#include "tesseract_collision/fcl/fcl_discrete_managers.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

template <typename T>
void addCollisionObjects(T& checker, bool use_convex_mesh = false)
{
  //////////////////////
  // Add box to checker
  //////////////////////
  shapes::ShapePtr box(new shapes::Box(1, 1, 1));
  Eigen::Isometry3d box_pose;
  box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  tesseract::VectorIsometry3d obj1_poses;
  tesseract::CollisionObjectTypeVector obj1_types;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);
  obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

  /////////////////////////////////////////////
  // Add thin box to checker which is disabled
  /////////////////////////////////////////////
  shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
  Eigen::Isometry3d thin_box_pose;
  thin_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  tesseract::VectorIsometry3d obj2_poses;
  tesseract::CollisionObjectTypeVector obj2_types;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);
  obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types, false);

  /////////////////////////////////////////////////////////////////
  // Add second box to checker. If use_convex_mesh = true then this
  // box will be added as a convex hull mesh.
  /////////////////////////////////////////////////////////////////
  shapes::ShapePtr second_box;
  if (use_convex_mesh)
    second_box.reset(shapes::createMeshFromResource("package://tesseract_collision/test/box_2m.stl"));
  else
    second_box.reset(new shapes::Box(2, 2, 2));

  Eigen::Isometry3d second_box_pose;
  second_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  tesseract::VectorIsometry3d obj3_poses;
  tesseract::CollisionObjectTypeVector obj3_types;
  obj3_shapes.push_back(second_box);
  obj3_poses.push_back(second_box_pose);
  if (use_convex_mesh)
    obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);
  else
    obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

  checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses, obj3_types);
}

void runDiscreteTest(tesseract::DiscreteContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  tesseract::ContactRequest req;
  req.link_names.push_back("box_link");
  req.link_names.push_back("second_box_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;
  checker.setContactRequest(req);

  // Set the collision object transforms
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Isometry3d::Identity();
  location["box_link"].translation()(0) = 0.2;
  location["box_link"].translation()(1) = 0.1;
  location["second_box_link"] = Eigen::Isometry3d::Identity();

  checker.setCollisionObjectsTransform(location);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -1.30, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], -0.3, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);

  ////////////////////////////////////////////////
  // Test object is out side the contact distance
  ////////////////////////////////////////////////
  location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
  result.clear();
  result_vector.clear();

  checker.setCollisionObjectsTransform(location);
  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  /////////////////////////////////////////////
  // Test object inside the contact distance
  /////////////////////////////////////////////
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.25;

  checker.setContactRequest(req);
  checker.contactTest(result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, 0.1, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], 1.1, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

void runContinuousTest(tesseract::ContinuousContactManagerBase& checker)
{
  //////////////////////////////////////
  // Test when object is inside another
  //////////////////////////////////////
  checker.enableCollisionObject("second_box_link");
  checker.enableCollisionObject("thin_box_link");
  checker.disableCollisionObject("box_link");

  tesseract::ContactRequest req;
  req.link_names.push_back("second_box_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::CLOSEST;
  checker.setContactRequest(req);

  // Set the collision object transforms
  tesseract::TransformMap location, location1, location2;
  // Set static object transforms
  location["thin_box_link"] = Eigen::Isometry3d::Identity();
  // Set moving object transforms
  location1["second_box_link"] = Eigen::Isometry3d::Identity();
  location1["second_box_link"].translation()(1) = -5.0;
  location2["second_box_link"] = Eigen::Isometry3d::Identity();
  location2["second_box_link"].translation()(1) = 5.0;

  checker.setCollisionObjectsTransform(location);
  checker.setCollisionObjectsTransform(location1, location2);

  // Perform collision check
  tesseract::ContactResultMap result;
  checker.contactTest(result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(!result_vector.empty());
  EXPECT_NEAR(result_vector[0].distance, -1.30, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][1], result_vector[0].nearest_points[1][1], 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[0][2], result_vector[0].nearest_points[1][2], 0.001);

  std::vector<int> idx = { 0, 1, 1 };
  if (result_vector[0].link_names[0] != "second_box_link")
    idx = { 1, 0, -1 };

  EXPECT_NEAR(result_vector[0].nearest_points[idx[0]][0], -0.3, 0.001);
  EXPECT_NEAR(result_vector[0].nearest_points[idx[1]][0], 1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[0], idx[2] * -1.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[1], idx[2] * 0.0, 0.001);
  EXPECT_NEAR(result_vector[0].normal[2], idx[2] * 0.0, 0.001);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxBoxUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker, false);
  runDiscreteTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteSimpleCollisionBoxBoxConvexHullUnit)
{
  tesseract::BulletDiscreteSimpleManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker, true);
  runDiscreteTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxBoxUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker, false);
  runDiscreteTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHCollisionBoxBoxConvexHullUnit)
{
  tesseract::BulletDiscreteBVHManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker, true);
  runDiscreteTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxBoxUnit)
{
  tesseract::FCLDiscreteBVHManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker, false);
  runDiscreteTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHCollisionBoxBoxConvexHullUnit)
{
  tesseract::FCLDiscreteBVHManager checker;
  addCollisionObjects<tesseract::DiscreteContactManagerBase>(checker, true);
  runDiscreteTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionBoxBoxUnit)
{
  tesseract::BulletCastSimpleManager checker;
  addCollisionObjects<tesseract::ContinuousContactManagerBase>(checker, false);
  runContinuousTest(checker);
}

TEST(TesseractCollisionUnit, BulletContinuousSimpleCollisionBoxBoxConvexHullUnit)
{
  tesseract::BulletCastSimpleManager checker;
  addCollisionObjects<tesseract::ContinuousContactManagerBase>(checker, true);
  runContinuousTest(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
