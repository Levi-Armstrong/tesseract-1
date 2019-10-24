#include <Eigen/Eigen>
#include <iostream>
#include <math.h>

//Just for plotting
#include <tesseract_ros_examples/basic_cartesian_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM =
    "robot_description_semantic"; /**< Default ROS parameter for robot description */

#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <trajopt/utils.hpp>
#include <tesseract_motion_planners/core/utils.h>
#include <numeric>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pubsurface;

Eigen::Isometry3d calcRotation(double rx, double ry, double rz)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
  pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
  pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));
  return pose;
}

//std::vector<Eigen::Isometry3d> getSlerp(Eigen::Is)

std::vector<tesseract_common::VectorIsometry3d> getSlerpEdges(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  std::vector<tesseract_common::VectorIsometry3d> edges;

  // X edges
  {
    auto rot1 = calcRotation(x_min, y_min, z_min);
    auto rot2 = calcRotation(x_max, y_min, z_min);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_min, y_min, z_max);
    auto rot2 = calcRotation(x_max, y_min, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_min, y_max, z_min);
    auto rot2 = calcRotation(x_max, y_max, z_min);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_min, y_max, z_max);
    auto rot2 = calcRotation(x_max, y_max, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  // Y Edges
  {
    auto rot1 = calcRotation(x_min, y_min, z_min);
    auto rot2 = calcRotation(x_min, y_max, z_min);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_min, y_min, z_max);
    auto rot2 = calcRotation(x_min, y_max, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_max, y_min, z_min);
    auto rot2 = calcRotation(x_max, y_max, z_min);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_max, y_min, z_max);
    auto rot2 = calcRotation(x_max, y_max, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  // Z Edges
  {
    auto rot1 = calcRotation(x_min, y_min, z_min);
    auto rot2 = calcRotation(x_min, y_min, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_min, y_max, z_min);
    auto rot2 = calcRotation(x_min, y_max, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_max, y_min, z_min);
    auto rot2 = calcRotation(x_max, y_min, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  {
    auto rot1 = calcRotation(x_max, y_max, z_min);
    auto rot2 = calcRotation(x_max, y_max, z_max);
    edges.push_back(tesseract_motion_planners::interpolate(rot1, rot2, 20));
  }
  return edges;
}

pcl::PointCloud<pcl::PointNormal>::Ptr generateSDF()
{
  double rx_res = 1;
  double ry_res = 1;
  double rz_res = 1;

  double rx_min = -20;
  double rx_max = 20;
  double ry_min = -20;
  double ry_max = 20;
  double rz_min = -20;
  double rz_max = 20;

  int rx_cnt = std::ceil(std::abs(rx_max - rx_min) / rx_res) + 1;
  int ry_cnt = std::ceil(std::abs(ry_max - ry_min) / ry_res) + 1;
  int rz_cnt = std::ceil(std::abs(rz_max - rz_min) / rz_res) + 1;

  Eigen::VectorXd rx_range = Eigen::VectorXd::LinSpaced(rx_cnt, rx_min, rx_max);
  Eigen::VectorXd ry_range = Eigen::VectorXd::LinSpaced(ry_cnt, ry_min, ry_max);
  Eigen::VectorXd rz_range = Eigen::VectorXd::LinSpaced(rz_cnt, rz_min, rz_max);

  // first generate surface
  pcl::PointCloud<pcl::PointXYZ> pc_surface;
  for (long ix = 0; ix < rx_range.size(); ++ix)
  {
    double rx = rx_range(ix);
    for (long iy = 0; iy < ry_range.size(); ++iy)
    {
      double ry = ry_range(iy);
      for (long iz = 0; iz < rz_range.size(); ++iz)
      {
        double rz = rz_range(iz);
        if (ix == 0 || ix == (rx_range.size() - 1) || iy == 0 || iy == (ry_range.size() - 1) || iz == 0 || iz == (rz_range.size() - 1))
        {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
          pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
          pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));

          // Convert to angle axis
          Eigen::Vector3d pnt = trajopt::calcRotationalError(pose.linear().matrix());
          pc_surface.push_back (pcl::PointXYZ(pnt(0), pnt(1), pnt(2)));
        }
      }
    }
  }
  pc_surface.header.frame_id = "world";
  pcl_conversions::toPCL(ros::Time::now(), pc_surface.header.stamp);
  pubsurface.publish (pc_surface);

  rx_res = 5;
  ry_res = 5;
  rz_res = 5;

  rx_cnt = std::ceil(std::abs(rx_max - rx_min) / rx_res) + 1;
  ry_cnt = std::ceil(std::abs(ry_max - ry_min) / ry_res) + 1;
  rz_cnt = std::ceil(std::abs(rz_max - rz_min) / rz_res) + 1;

  rx_range = Eigen::VectorXd::LinSpaced(rx_cnt, rx_min, rx_max);
  ry_range = Eigen::VectorXd::LinSpaced(ry_cnt, ry_min, ry_max);
  rz_range = Eigen::VectorXd::LinSpaced(rz_cnt, rz_min, rz_max);

  pcl::PointCloud<pcl::PointNormal>::Ptr sign_distance_field(new pcl::PointCloud<pcl::PointNormal>());
  sign_distance_field->reserve(rx_cnt * ry_cnt * rz_cnt);
  for (long ix = 0; ix < rx_range.size(); ++ix)
  {
    double rx = rx_range(ix);
    for (long iy = 0; iy < ry_range.size(); ++iy)
    {
      double ry = ry_range(iy);
      for (long iz = 0; iz < rz_range.size(); ++iz)
      {
        double rz = rz_range(iz);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
        pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
        pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));

        // Convert to angle axis
        Eigen::Vector3d pnt = trajopt::calcRotationalError(pose.linear().matrix());
        pcl::PointNormal closest;
        closest.x = pnt.x();
        closest.y = pnt.y();
        closest.z = pnt.z();

        Eigen::Quaterniond cq(pose.linear().matrix());
        double dist = std::numeric_limits<double>::max();
        for (auto& p : pc_surface.points)
        {
          Eigen::Vector3d a(p.x, p.y, p.z);
          Eigen::Quaterniond qe(Eigen::AngleAxisd(a.norm(), a.normalized()));
          double d = 1 - std::pow(cq.w() * qe.w() + cq.x() * qe.x() + cq.y() * qe.y() + cq.z() * qe.z(), 2);
          if (d < dist)
          {
            closest.normal_x = a.x();
            closest.normal_y = a.y();
            closest.normal_z = a.z();
            closest.curvature = -d;
            dist = d;
          }
        }

        sign_distance_field->push_back (closest);
      }
    }
  }

  rx_cnt = std::ceil(std::abs(rx_min - (-180)) / rx_res) + 1;
  ry_cnt = std::ceil(std::abs(ry_min - (-180)) / ry_res) + 1;
  rz_cnt = std::ceil(std::abs(rz_min - (-180)) / rz_res) + 1;

  rx_range = Eigen::VectorXd::LinSpaced(rx_cnt, -180, rx_min);
  ry_range = Eigen::VectorXd::LinSpaced(ry_cnt, -180, ry_min);
  rz_range = Eigen::VectorXd::LinSpaced(rz_cnt, -180, rz_min);

  sign_distance_field->reserve(sign_distance_field->size() + (rx_cnt * ry_cnt * rz_cnt));
  for (long ix = 0; ix < rx_range.size() - 1; ++ix)
  {
    double rx = rx_range(ix);
    for (long iy = 0; iy < ry_range.size() - 1; ++iy)
    {
      double ry = ry_range(iy);
      for (long iz = 0; iz < rz_range.size() -1; ++iz)
      {
        double rz = rz_range(iz);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
        pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
        pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));

        // Convert to angle axis
        Eigen::Vector3d pnt = trajopt::calcRotationalError(pose.linear().matrix());
        pcl::PointNormal closest;
        closest.x = pnt.x();
        closest.y = pnt.y();
        closest.z = pnt.z();

        Eigen::Quaterniond cq(pose.linear().matrix());
        double dist = std::numeric_limits<double>::max();
        for (auto& p : pc_surface.points)
        {
          Eigen::Vector3d a(p.x, p.y, p.z);
          Eigen::Quaterniond qe(Eigen::AngleAxisd(a.norm(), a.normalized()));
          double d = 1 - std::pow(cq.w() * qe.w() + cq.x() * qe.x() + cq.y() * qe.y() + cq.z() * qe.z(), 2);
          if (d < dist)
          {
            closest.normal_x = a.x();
            closest.normal_y = a.y();
            closest.normal_z = a.z();
            closest.curvature = d;
            dist = d;
          }
        }

        sign_distance_field->push_back (closest);
      }
    }
  }

  rx_cnt = std::ceil(std::abs(180 - rx_max) / rx_res) + 1;
  ry_cnt = std::ceil(std::abs(180 - ry_max) / ry_res) + 1;
  rz_cnt = std::ceil(std::abs(180 - rz_max) / rz_res) + 1;

  rx_range = Eigen::VectorXd::LinSpaced(rx_cnt, rx_max, 180);
  ry_range = Eigen::VectorXd::LinSpaced(ry_cnt, ry_max, 180);
  rz_range = Eigen::VectorXd::LinSpaced(rz_cnt, rz_max, 180);

  sign_distance_field->reserve(sign_distance_field->size() + (rx_cnt * ry_cnt * rz_cnt));
  for (long ix = 1; ix < rx_range.size(); ++ix)
  {
    double rx = rx_range(ix);
    for (long iy = 1; iy < ry_range.size(); ++iy)
    {
      double ry = ry_range(iy);
      for (long iz = 1; iz < rz_range.size(); ++iz)
      {
        double rz = rz_range(iz);

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
        pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
        pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));

        // Convert to angle axis
        Eigen::Vector3d pnt = trajopt::calcRotationalError(pose.linear().matrix());
        pcl::PointNormal closest;
        closest.x = pnt.x();
        closest.y = pnt.y();
        closest.z = pnt.z();

        Eigen::Quaterniond cq(pose.linear().matrix());
        double dist = std::numeric_limits<double>::max();
        for (auto& p : pc_surface.points)
        {
          Eigen::Vector3d a(p.x, p.y, p.z);
          Eigen::Quaterniond qe(Eigen::AngleAxisd(a.norm(), a.normalized()));
          double d = 1 - std::pow(cq.w() * qe.w() + cq.x() * qe.x() + cq.y() * qe.y() + cq.z() * qe.z(), 2);
          if (d < dist)
          {
            closest.normal_x = a.x();
            closest.normal_y = a.y();
            closest.normal_z = a.z();
            closest.curvature = d;
            dist = d;
          }
        }

        sign_distance_field->push_back (closest);
      }
    }
  }

  return sign_distance_field;
}

int main(int argc, char** argv)
{
  // All of this is just to use the rviz plot axis tool
  ros::init(argc, argv, "rotation_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  tesseract_scene_graph::ResourceLocatorFn locator = tesseract_rosutils::locateResource;
  auto tesseract = std::make_shared<tesseract::Tesseract>();
  if (!tesseract->init(urdf_xml_string, srdf_xml_string, locator))
    return -1;
  ros::Publisher pub1 = nh.advertise<PointCloud> ("points1", 1);
  ros::Publisher pub2 = nh.advertise<PointCloud> ("points2", 1);
  ros::Publisher pub3 = nh.advertise<PointCloud> ("points3", 1);
  pubsurface = nh.advertise<PointCloud> ("surface", 1);

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter = std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract->getEnvironment());
  plotter->waitForInput();

  pcl::PointCloud<pcl::PointNormal>::Ptr sdf = generateSDF();
  pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
  kdtree.setInputCloud (sdf);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "world";
  msg->width = 1;

  Eigen::Isometry3d check_pose = Eigen::Isometry3d::Identity();
  check_pose.rotate(Eigen::AngleAxisd(22 * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
  check_pose.rotate(Eigen::AngleAxisd(0 * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
  check_pose.rotate(Eigen::AngleAxisd(95 * M_PI / 180., Eigen::Vector3d(1, 0, 0)));
  Eigen::Quaterniond cq(check_pose.linear().matrix());
  Eigen::Vector3d check_err = trajopt::calcRotationalError(check_pose.linear().matrix());

  pcl::PointNormal searchPoint;
  searchPoint.x = check_err.x();
  searchPoint.y = check_err.y();
  searchPoint.z = check_err.z();


  plotter->plotAxis(check_pose, 0.6);

  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
  pcl::PointNormal closet = sdf->at(pointIdxNKNSearch[0]);
  std::cout << closet << std::endl;

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Vector3d a(closet.normal_x, closet.normal_y, closet.normal_z);
  target.linear() = Eigen::AngleAxisd(a.norm(), a.normalized()).matrix();
  plotter->plotAxis(target, 1.0);

  Eigen::Isometry3d mid_target = Eigen::Isometry3d::Identity();
  Eigen::Vector3d mid_a(a.x() + ((check_err(0) - a.x())/2.0), a.y() + ((check_err(1) - a.y())/2.0), a.z() + ((check_err(2) - a.z())/2.0));
  mid_target.linear() = Eigen::AngleAxisd(mid_a.norm(), mid_a.normalized()).matrix();
  plotter->plotAxis(mid_target, .8);

  PointCloud::Ptr check_msg (new PointCloud);
  check_msg->header.frame_id = "world";
  pcl_conversions::toPCL(ros::Time::now(), check_msg->header.stamp);
  check_msg->width = 1;
  check_msg->height = 3;
  check_msg->points.resize(3);
  check_msg->points[0] = pcl::PointXYZ(check_err(0), check_err(1), check_err(2));
  check_msg->points[1] = pcl::PointXYZ(mid_a(0), mid_a(1), mid_a(2));
  check_msg->points[2] = pcl::PointXYZ(closet.normal_x, closet.normal_y, closet.normal_z);
  pub3.publish (check_msg);

//  // Check angle axis
//  Eigen::Isometry3d pp1 = Eigen::Isometry3d::Identity();
//  pp1.rotate(Eigen::AngleAxisd(5 * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
//  Eigen::Vector3d ee1 = trajopt::calcRotationalError(pp1.linear().matrix());
//  std::cout << ee1 << std::endl;

//  Eigen::Isometry3d pp2 = Eigen::Isometry3d::Identity();
//  pp2.rotate(Eigen::AngleAxisd(-5 * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
//  Eigen::Vector3d ee2 = trajopt::calcRotationalError(pp2.linear().matrix());
//  std::cout << ee2 << std::endl;

//  double d = 1 - std::pow(ee1.dot(ee2), 2);
//  std::cout << d << std::endl;

//  // Here is the actual example
//  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
//  plotter->plotAxis(target, 1.0);

//  Eigen::Vector3d closet;
//  double dist = 10000000000;
//  double check_dist = dist;
//  // Loop over some rotations and check that the error is negative if they are less than the tolerances
//  double rx_res = 9.99;
//  double ry_res = 9.99;
//  double rz_res = 9.99;
//  for (double rx = -90; rx < 90; rx += rx_res)
//  {
//    for (double ry = -90; ry < 90; ry += ry_res)
//    {
//      for (double rz = -90; rz < 90; rz += rz_res)
//      {
////        double rz = 0.00001;
//        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//        pose.rotate(Eigen::AngleAxisd(rz * M_PI / 180., Eigen::Vector3d(0, 0, 1)));
//        pose.rotate(Eigen::AngleAxisd(ry * M_PI / 180., Eigen::Vector3d(0, 1, 0)));
//        pose.rotate(Eigen::AngleAxisd(rx * M_PI / 180., Eigen::Vector3d(1, 0, 0)));

//        Eigen::Isometry3d pose_delta = pose;
//        // Convert to angle axis
//        Eigen::Vector3d pnt = trajopt::calcRotationalError(pose_delta.linear().matrix());
//        msg->points.push_back (pcl::PointXYZ(pnt(0),pnt(1),pnt(2)));

//        // Convert to quaternion back projection
//        Eigen::Quaterniond qe(pose_delta.linear().matrix());
//        double cd = 1 - std::pow(cq.w() * qe.w() + cq.x() * qe.x() + cq.y() * qe.y() + cq.z() * qe.z(), 2);
//        if (cd < dist)
//        {
//          double angle_axis_norm = trajopt::calcRotationalError((pose_delta.inverse() * check_pose).linear().matrix()).norm();
//          if (angle_axis_norm < check_dist)
//            check_dist = angle_axis_norm;
//          else
//            assert(false);

//          closet = pnt;
//          dist = cd;
//          check_msg->points[1] = pcl::PointXYZ(pnt(0), pnt(1), pnt(2));
//        }
////        double a = (1-qe.z())/(qe.z() + 1);
////        double xp = qe.w() * (a + 1)/2.0;
////        double yp = qe.x() * (a + 1)/2.0;
////        double zp = qe.y() * (a + 1)/2.0;
////        msg->points.push_back (pcl::PointXYZ(xp,yp,zp));


//        // eulerAngles(2,1,0) returns the rotation in ZYX Euler angles. Since these can be converted to fixed axis
//        // rotations by reversing the order, this corresponds to XYZ rotations about the fixed target axis
//        Eigen::Vector3d euler_delta = pose_delta.linear().eulerAngles(2, 1, 0);
//        Eigen::Vector3d fixed_delta(euler_delta(2), euler_delta(1), euler_delta(0));
//        Eigen::Vector3d fixed_delta_degrees = fixed_delta * 180. / M_PI;

//        Eigen::Matrix3d rot = pose_delta.rotation().matrix();
//        double rx_out = atan2(rot(2, 1), rot(2, 2)) * 180. / M_PI;
//        double ry_out = atan2(-rot(2, 0), sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2))) * 180. / M_PI;
//        double rz_out = atan2(rot(1, 0), rot(0, 0)) * 180. / M_PI;
//        Eigen::Vector3d fixed_delta_degrees2(rx_out, ry_out, rz_out);

//        Eigen::Vector3d out = fixed_delta_degrees;
//        if(out(0) > -10 && out(0) <10 && out(1) > -45 && out(1) < 45 && out(2) > 0 && out(2) < 5)
//          plotter->plotAxis(pose, 0.5);


////        if ( std::abs(out(0)-rx) > 1e-5 || std::abs(out(1)-ry) > 1e-5 || std::abs(out(2)-rz) > 1e-5)
////          std::cout << "rx_in/rx_out: " << rx << "/" << out(0) << "  ry_in/ry_out:" << ry << "/" << out(1) << "  rz_in/rz_out:" << rz << "/" << out(2) << std::endl;
//      }
//    }
//  }
//  msg->height = msg->points.size();
//  ROS_INFO("Cloud2 Points: %d", msg->points.size());
//  ROS_INFO("Cloud3 Points: %d", check_msg->points.size());
//  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
//  pcl_conversions::toPCL(ros::Time::now(), check_msg->header.stamp);
//  pub2.publish (msg);
//  pub3.publish (check_msg);

//  // Plot slerp edges to compare
//  PointCloud::Ptr edge_msg (new PointCloud);
//  edge_msg->header.frame_id = "world";
//  edge_msg->width = 1;

//  auto edges = getSlerpEdges(-45, 45, -45, 45, 0.001, 0.001);
//  for (auto& edge : edges)
//  {
//    for(auto& rot : edge)
//    {
//      Eigen::Vector3d pnt = trajopt::calcRotationalError(rot.linear().matrix());
//      edge_msg->points.push_back (pcl::PointXYZ(pnt(0),pnt(1),pnt(2)));
//    }
//    edge_msg->height = edge_msg->points.size();
//    pcl_conversions::toPCL(ros::Time::now(), edge_msg->header.stamp);
//    pub1.publish (edge_msg);
//  }
//  edge_msg->height = edge_msg->points.size();
//  ROS_INFO("Cloud1 Points: %d", edge_msg->points.size());
//  pcl_conversions::toPCL(ros::Time::now(), edge_msg->header.stamp);
//  pub1.publish (edge_msg);



  ROS_WARN("Done");
  ros::spin();
}
