#pragma once
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using PointType = pcl::PointXYZI;

double inline toSec(const rclcpp::Time &timestamp) {
  return static_cast<double>(timestamp.seconds()) +
         1e-9 * static_cast<double>(timestamp.nanoseconds());
};

double inline toSec(const builtin_interfaces::msg::Time &timestamp) {
  return static_cast<double>(timestamp.sec) +
         1e-9 * static_cast<double>(timestamp.nanosec);
};

rclcpp::Time inline fromSec(const double t) {
  auto sec = (uint32_t)floor(t);
  return rclcpp::Time(sec, (uint32_t)std::round((t - sec) * 1e9));
}

inline void matrixEigenToTF(const Eigen::Matrix3d &e, tf2::Matrix3x3 &t) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      t[i][j] = e(i, j);
}

inline void matrixTFToEigen(const tf2::Matrix3x3 &t, Eigen::Matrix3d &e) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      e(i, j) = t[i][j];
}

inline pcl::PointCloud<PointType>::Ptr
voxelizePcd(const pcl::PointCloud<PointType> &pcd_in, const float voxel_res) {
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  pcd_in_ptr->reserve(pcd_in.size());
  pcd_out->reserve(pcd_in.size());
  *pcd_in_ptr = pcd_in;
  voxelgrid.setInputCloud(pcd_in_ptr);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr
voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
            const float voxel_res) {
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  pcd_out->reserve(pcd_in->size());
  voxelgrid.setInputCloud(pcd_in);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

inline gtsam::Pose3 poseEigToGtsamPose(const Eigen::Matrix4d &pose_eig_in) {
  double r, p, y;
  tf2::Matrix3x3 mat;
  matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
  mat.getRPY(r, p, y);
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(r, p, y),
      gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}

inline Eigen::Matrix4d gtsamPoseToPoseEig(const gtsam::Pose3 &gtsam_pose_in) {
  Eigen::Matrix4d pose_eig_out = Eigen::Matrix4d::Identity();
  // Eigen::Quaterniond quat =
  // Eigen::AngleAxisd(gtsam_pose_in.rotation().roll(),
  //                                             Eigen::Vector3d::UnitX()) *
  //                           Eigen::AngleAxisd(gtsam_pose_in.rotation().pitch(),
  //                                             Eigen::Vector3d::UnitY()) *
  //                           Eigen::AngleAxisd(gtsam_pose_in.rotation().yaw(),
  //                                             Eigen::Vector3d::UnitZ());

  tf2::Quaternion quat;
  quat.setRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(),
              gtsam_pose_in.rotation().yaw());
  tf2::Matrix3x3 mat(quat);
  Eigen::Matrix3d tmp_rot_mat;
  matrixTFToEigen(mat, tmp_rot_mat);

  pose_eig_out.block<3, 3>(0, 0) = tmp_rot_mat;
  pose_eig_out(0, 3) = gtsam_pose_in.translation().x();
  pose_eig_out(1, 3) = gtsam_pose_in.translation().y();
  pose_eig_out(2, 3) = gtsam_pose_in.translation().z();
  return pose_eig_out;
}

inline geometry_msgs::msg::PoseStamped
poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                     std::string frame_id = "map") {
  double r, p, y;
  tf2::Matrix3x3 mat;
  matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
  mat.getRPY(r, p, y);

  tf2::Quaternion quat;
  quat.setRPY(r, p, y);
  // Eigen::Quaterniond quat = Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) *
  //                           Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
  //                           Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = pose_eig_in(0, 3);
  pose.pose.position.y = pose_eig_in(1, 3);
  pose.pose.position.z = pose_eig_in(2, 3);
  pose.pose.orientation.w = quat.w();
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  return pose;
}

inline tf2::Transform poseEigToROSTf(const Eigen::Matrix4d &pose) {
  Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  transform.setRotation(
      tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
  return transform;
}

inline tf2::Transform
poseStampedToROSTf(const geometry_msgs::msg::PoseStamped &pose) {
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.pose.position.x, pose.pose.position.y,
                                   pose.pose.position.z));
  transform.setRotation(
      tf2::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w));
  return transform;
}

inline geometry_msgs::msg::PoseStamped
gtsamPoseToPoseStamped(const gtsam::Pose3 &gtsam_pose_in,
                       std::string frame_id = "map") {
  // Eigen::Quaterniond quat =
  // Eigen::AngleAxisd(gtsam_pose_in.rotation().roll(),
  //                                             Eigen::Vector3d::UnitX()) *
  //                           Eigen::AngleAxisd(gtsam_pose_in.rotation().pitch(),
  //                                             Eigen::Vector3d::UnitY()) *
  //                           Eigen::AngleAxisd(gtsam_pose_in.rotation().yaw(),
  //                                             Eigen::Vector3d::UnitZ());

  tf2::Quaternion quat;
  quat.setRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(),
              gtsam_pose_in.rotation().yaw());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = gtsam_pose_in.translation().x();
  pose.pose.position.y = gtsam_pose_in.translation().y();
  pose.pose.position.z = gtsam_pose_in.translation().z();
  pose.pose.orientation.w = quat.w();
  pose.pose.orientation.x = quat.x();
  pose.pose.orientation.y = quat.y();
  pose.pose.orientation.z = quat.z();
  return pose;
}

template <typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud,
                                                 std::string frame_id = "map") {
  sensor_msgs::msg::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf) {
  if (cloud_in.size() == 0) {
    return cloud_in;
  }
  pcl::PointCloud<T> pcl_out = cloud_in;
  pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
  return pcl_out;
}