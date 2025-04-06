#pragma once

#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/logging.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "loop_closure.hpp"
#include "pose_pcd.hpp"
#include "utilities.hpp"

#ifdef FASTLIO_SAM_QN_PARAM_DEBUG
#define GET_PARAM_DEBUG(name, param)                                           \
  this->get_parameter(name, param);                                            \
  RCLCPP_INFO_STREAM(this->get_logger(), name << ": " << param);
#else
#define GET_PARAM_DEBUG(name, param) this->get_parameter(name, param);
#endif

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>
    odom_pcd_sync_pol;

class FastLioSamQn : public rclcpp::Node {
public:
  using PointCloudT = sensor_msgs::msg::PointCloud2;
  using PathT = nav_msgs::msg::Path;
  using MarkerT = visualization_msgs::msg::Marker;
  using PoseStampedT = geometry_msgs::msg::PoseStamped;
  using OdomT = nav_msgs::msg::Odometry;
  using StringT = std_msgs::msg::String;

private:
  ///// basic params
  std::string map_frame_;
  std::string package_path_;
  std::string seq_name_;
  ///// shared data - odom and pcd
  std::mutex realtime_pose_mutex_, keyframes_mutex_;
  std::mutex graph_mutex_, vis_mutex_;
  Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
  PosePcd current_frame_;
  std::vector<PosePcd> keyframes_;
  int current_keyframe_idx_ = 0;
  ///// graph and values
  bool is_initialized_ = false;
  bool loop_added_flag_ = false;     // for opt
  bool loop_added_flag_vis_ = false; // for vis
  std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
  gtsam::NonlinearFactorGraph gtsam_graph_;
  gtsam::Values init_esti_;
  gtsam::Values corrected_esti_;
  double keyframe_thr_;
  double voxel_res_;
  int sub_key_num_;
  std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; // for vis
  ///// visualize
  tf2_ros::Buffer tfListener_buffer_;
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::TransformListener tfListener_;
  pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
  nav_msgs::msg::Path odom_path_, corrected_path_;
  bool global_map_vis_switch_ = true;
  ///// results
  bool save_map_bag_ = false, save_map_pcd_ = false,
       save_in_kitti_format_ = false;
  ///// ros
  //   ros::NodeHandle nh_;
  //   ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_,
  //   path_pub_; ros::Publisher corrected_current_pcd_pub_,
  //   corrected_pcd_map_pub_,
  //       loop_detection_pub_;
  //   ros::Publisher realtime_pose_pub_;
  //   ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_,
  //       debug_fine_aligned_pub_;
  //   ros::Subscriber sub_save_flag_;
  rclcpp::Publisher<PointCloudT>::SharedPtr odom_pub_;
  rclcpp::Publisher<PathT>::SharedPtr path_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<PathT>::SharedPtr corrected_path_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr corrected_pcd_map_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr corrected_current_pcd_pub_;
  rclcpp::Publisher<MarkerT>::SharedPtr loop_detection_pub_;
  rclcpp::Publisher<PoseStampedT>::SharedPtr realtime_pose_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_dst_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_coarse_aligned_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_fine_aligned_pub_;

  rclcpp::Subscription<StringT>::SharedPtr sub_save_flag_;

  rclcpp::TimerBase::SharedPtr loop_timer_, vis_timer_;
  // odom, pcd sync, and save flag subscribers
  std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>>
      sub_odom_pcd_sync_ = nullptr;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>
      sub_odom_ = nullptr;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      sub_pcd_ = nullptr;
  ///// Loop closure
  std::shared_ptr<LoopClosure> loop_closure_;

public:
  explicit FastLioSamQn()
      : Node("FastLioSamQn"), tfListener_buffer_(this->get_clock()),
        broadcaster_(*this), tfListener_(tfListener_buffer_) {
    double loop_update_hz, vis_hz;
    LoopClosureConfig lc_config;
    auto &gc = lc_config.gicp_config_;
    auto &qc = lc_config.quatro_config_;
    /* basic */
    this->declare_parameter("basic.map_frame", map_frame_);
    this->declare_parameter("basic.loop_update_hz", loop_update_hz);
    this->declare_parameter("basic.vis_hz", vis_hz);
    this->declare_parameter("save_voxel_resolution", voxel_res_);
    this->declare_parameter("quatro_nano_gicp_voxel_resolution",
                            lc_config.voxel_res_);
    /* keyframe */
    this->declare_parameter("keyframe.keyframe_threshold", keyframe_thr_);
    this->declare_parameter("keyframe.nusubmap_keyframes",
                            lc_config.num_submap_keyframes_);
    this->declare_parameter("keyframe.enable_submap_matching",
                            lc_config.enable_submap_matching_);
    /* loop */
    this->declare_parameter("loop.loop_detection_radius",
                            lc_config.loop_detection_radius_);
    this->declare_parameter("loop.loop_detection_timediff_threshold",
                            lc_config.loop_detection_timediff_threshold_);

    /* nano (GICP config) */
    this->declare_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
    this->declare_parameter("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
    this->declare_parameter("nano_gicp.correspondences_number",
                            gc.nano_correspondences_number_);
    this->declare_parameter("nano_gicp.max_iter", gc.nano_max_iter_);
    this->declare_parameter("nano_gicp.transformation_epsilon",
                            gc.transformation_epsilon_);
    this->declare_parameter("nano_gicp.euclidean_fitness_epsilon",
                            gc.euclidean_fitness_epsilon_);
    this->declare_parameter("nano_gicp.ransac.max_iter",
                            gc.nano_ransac_max_iter_);
    this->declare_parameter("nano_gicp.ransac.outlier_rejection_threshold",
                            gc.ransac_outlier_rejection_threshold_);
    /* quatro (Quatro config) */
    this->declare_parameter("quatro.enable", lc_config.enable_quatro_);
    this->declare_parameter("quatro.optimize_matching",
                            qc.use_optimized_matching_);
    this->declare_parameter("quatro.distance_threshold",
                            qc.quatro_distance_threshold_);
    this->declare_parameter("quatro.max_nucorrespondences",
                            qc.quatro_max_num_corres_);
    this->declare_parameter("quatro.fpfh_normal_radius",
                            qc.fpfh_normal_radius_);
    this->declare_parameter("quatro.fpfh_radius", qc.fpfh_radius_);
    this->declare_parameter("quatro.estimating_scale", qc.estimat_scale_);
    this->declare_parameter("quatro.noise_bound", qc.noise_bound_);
    this->declare_parameter("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
    this->declare_parameter("quatro.rotation.rot_cost_diff_threshold",
                            qc.rot_cost_diff_thr_);
    this->declare_parameter("quatro.rotation.numax_iter", qc.quatro_max_iter_);
    /* results */
    this->declare_parameter("result.save_map_bag", save_map_bag_);
    this->declare_parameter("result.save_map_pcd", save_map_pcd_);
    this->declare_parameter("result.save_in_kitti_format",
                            save_in_kitti_format_);
    this->declare_parameter("result.seq_name", seq_name_);

    /* basic */
    GET_PARAM_DEBUG("basic.map_frame", map_frame_);
    GET_PARAM_DEBUG("basic.loop_update_hz", loop_update_hz);
    GET_PARAM_DEBUG("basic.vis_hz", vis_hz);
    GET_PARAM_DEBUG("save_voxel_resolution", voxel_res_);
    GET_PARAM_DEBUG("quatro_nano_gicp_voxel_resolution", lc_config.voxel_res_);
    /* keyframe */
    GET_PARAM_DEBUG("keyframe.keyframe_threshold", keyframe_thr_);
    GET_PARAM_DEBUG("keyframe.nusubmap_keyframes",
                    lc_config.num_submap_keyframes_);
    GET_PARAM_DEBUG("keyframe.enable_submap_matching",
                    lc_config.enable_submap_matching_);
    /* loop */
    GET_PARAM_DEBUG("loop.loop_detection_radius",
                    lc_config.loop_detection_radius_);
    GET_PARAM_DEBUG("loop.loop_detection_timediff_threshold",
                    lc_config.loop_detection_timediff_threshold_);

    /* nano (GICP config) */
    GET_PARAM_DEBUG("nano_gicp.thread_number", gc.nano_thread_number_);
    GET_PARAM_DEBUG("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
    GET_PARAM_DEBUG("nano_gicp.correspondences_number",
                    gc.nano_correspondences_number_);
    GET_PARAM_DEBUG("nano_gicp.max_iter", gc.nano_max_iter_);
    GET_PARAM_DEBUG("nano_gicp.transformation_epsilon",
                    gc.transformation_epsilon_);
    GET_PARAM_DEBUG("nano_gicp.euclidean_fitness_epsilon",
                    gc.euclidean_fitness_epsilon_);
    GET_PARAM_DEBUG("nano_gicp.ransac.max_iter", gc.nano_ransac_max_iter_);
    GET_PARAM_DEBUG("nano_gicp.ransac.outlier_rejection_threshold",
                    gc.ransac_outlier_rejection_threshold_);
    /* quatro (Quatro config) */
    GET_PARAM_DEBUG("quatro.enable", lc_config.enable_quatro_);
    GET_PARAM_DEBUG("quatro.optimize_matching", qc.use_optimized_matching_);
    GET_PARAM_DEBUG("quatro.distance_threshold", qc.quatro_distance_threshold_);
    GET_PARAM_DEBUG("quatro.max_nucorrespondences", qc.quatro_max_num_corres_);
    GET_PARAM_DEBUG("quatro.fpfh_normal_radius", qc.fpfh_normal_radius_);
    GET_PARAM_DEBUG("quatro.fpfh_radius", qc.fpfh_radius_);
    GET_PARAM_DEBUG("quatro.estimating_scale", qc.estimat_scale_);
    GET_PARAM_DEBUG("quatro.noise_bound", qc.noise_bound_);
    GET_PARAM_DEBUG("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
    GET_PARAM_DEBUG("quatro.rotation.rot_cost_diff_threshold",
                    qc.rot_cost_diff_thr_);
    GET_PARAM_DEBUG("quatro.rotation.numax_iter", qc.quatro_max_iter_);
    /* results */
    GET_PARAM_DEBUG("result.save_map_bag", save_map_bag_);
    GET_PARAM_DEBUG("result.save_map_pcd", save_map_pcd_);
    GET_PARAM_DEBUG("result.save_in_kitti_format", save_in_kitti_format_);
    GET_PARAM_DEBUG("result.seq_name", seq_name_);

    gc.max_corr_dist_ = lc_config.loop_detection_radius_ * 1.5;
    loop_closure_.reset(new LoopClosure(lc_config));
    /* Initialization of GTSAM */
    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;
    package_path_ =
        ament_index_cpp::get_package_share_directory("fast_lio_sam_qn");
    /* publishers */
    odom_pub_ = this->create_publisher<PointCloudT>("/ori_odom", 10);
    path_pub_ = this->create_publisher<PathT>("/ori_path", 10);
    corrected_odom_pub_ =
        this->create_publisher<PointCloudT>("/corrected_odom", 10);
    corrected_path_pub_ = this->create_publisher<PathT>("/corrected_path", 10);
    corrected_pcd_map_pub_ =
        this->create_publisher<PointCloudT>("/corrected_map", 10);
    corrected_current_pcd_pub_ =
        this->create_publisher<PointCloudT>("/corrected_current_pcd", 10);
    loop_detection_pub_ =
        this->create_publisher<MarkerT>("/loop_detection", 10);
    realtime_pose_pub_ =
        this->create_publisher<PoseStampedT>("/pose_stamped", 10);
    debug_src_pub_ = this->create_publisher<PointCloudT>("/src", 10);
    debug_dst_pub_ = this->create_publisher<PointCloudT>("/dst", 10);
    debug_coarse_aligned_pub_ =
        this->create_publisher<PointCloudT>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ =
        this->create_publisher<PointCloudT>("/fine_aligned_nano_gicp", 10);
    /* subscribers */
    rmw_qos_profile_t profile_ = rclcpp::QoS(10).get_rmw_qos_profile();
    sub_odom_ = std::make_shared<message_filters::Subscriber<OdomT>>(
        this, "/Odometry", profile_);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<PointCloudT>>(
        this, "/cloud_registered", profile_);
    sub_odom_pcd_sync_ =
        std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(
            odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(
        std::bind(&FastLioSamQn::odomPcdCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
    sub_save_flag_ = this->create_subscription<StringT>(
        "/save_dir", 1,
        std::bind(&FastLioSamQn::saveFlagCallback, this,
                  std::placeholders::_1));
    /* Timers */
    loop_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration(std::chrono::duration<double>(1 / loop_update_hz)),
        std::bind(&FastLioSamQn::loopTimerFunc, this));

    vis_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration(std::chrono::duration<double>(1 / vis_hz)),
        std::bind(&FastLioSamQn::visTimerFunc, this));

    RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
  }

  ~FastLioSamQn() {

    RCLCPP_INFO(this->get_logger(), "FastLioSam Exit and Saving...");
    // save map
    if (save_map_bag_) {
      rosbag2_cpp::Writer writer;
      rosbag2_storage::StorageOptions write_storage_options{};
      write_storage_options.uri = package_path_ + "/result.bag";
      write_storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options{};
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      writer.open(write_storage_options, converter_options);
      writer.create_topic(
          {"/keyframe_pcd", "sensor_msgs/msg/PointCloud2", "cdr", ""});
      writer.create_topic(
          {"/keyframe_pose", "geometry_msgs/msg/PoseStamped", "cdr", ""});
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
          rclcpp::Time time = fromSec(keyframes_[i].timestamp_);
          writer.write(pclToPclRos(keyframes_[i].pcd_, map_frame_),
                       "/keyframe_pcd", time);
          writer.write(poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_),
                       "/keyframe_pose", time);
        }
      }
      writer.close();
      RCLCPP_INFO(this->get_logger(),
                  "\033[36;1mResult saved in .bag format!!!\033[0m");
    }

    if (save_map_pcd_) {
      pcl::PointCloud<PointType>::Ptr corrected_map(
          new pcl::PointCloud<PointType>());
      corrected_map->reserve(keyframes_[0].pcd_.size() *
                             keyframes_.size()); // it's an approximated size
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
          *corrected_map += transformPcd(keyframes_[i].pcd_,
                                         keyframes_[i].pose_corrected_eig_);
        }
      }
      const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
      pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd",
                                           *voxelized_map);
      RCLCPP_INFO(this->get_logger(),
                  "\033[32;1mResult saved in .pcd format!!!\033[0m");
    }
  }

private:
  // methods
  void updateOdomsAndPaths(const PosePcd &pose_pcd_in) {
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(
        poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(
        poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
  }

  bool checkIfKeyframe(const PosePcd &pose_pcd_in,
                       const PosePcd &latest_pose_pcd) {
    return keyframe_thr_ <
           (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
            pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3))
               .norm();
  }

  visualization_msgs::msg::Marker
  getLoopMarkers(const gtsam::Values &corrected_esti_in) {
    MarkerT edges;
    edges.type = 5u;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i) {
      if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
          loop_idx_pairs_[i].second >= corrected_esti_in.size()) {
        continue;
      }
      gtsam::Pose3 pose =
          corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
      gtsam::Pose3 pose2 =
          corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
      geometry_msgs::msg::Point p, p2;
      p.x = pose.translation().x();
      p.y = pose.translation().y();
      p.z = pose.translation().z();
      p2.x = pose2.translation().x();
      p2.y = pose2.translation().y();
      p2.z = pose2.translation().z();
      edges.points.push_back(p);
      edges.points.push_back(p2);
    }
    return edges;
  }
  // cb
  void odomPcdCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg) {
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_; // to calculate delta
    current_frame_ =
        PosePcd(*odom_msg, *pcd_msg,
                current_keyframe_idx_); // to be checked if keyframe or not
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    {
      //// 1. realtime pose = last corrected odom * delta (last -> current)
      std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
      odom_delta_ =
          odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
      current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
      realtime_pose_pub_->publish(
          poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));

      geometry_msgs::msg::TransformStamped trans_stamped_msg_;
      trans_stamped_msg_.transform =
          tf2::toMsg(poseEigToROSTf(current_frame_.pose_corrected_eig_));
      trans_stamped_msg_.header.frame_id = map_frame_;
      trans_stamped_msg_.child_frame_id = "robot";
      trans_stamped_msg_.header.stamp = rclcpp::Clock().now();

      broadcaster_.sendTransform(trans_stamped_msg_);
    }
    corrected_current_pcd_pub_->publish(pclToPclRos(
        transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_),
        map_frame_));

    if (!is_initialized_) //// init only once
    {
      // others
      keyframes_.push_back(current_frame_);
      updateOdomsAndPaths(current_frame_);
      // graph
      auto variance_vector =
          (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2)
              .finished(); // rad*rad,
                           // meter*meter
      gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
          gtsam::noiseModel::Diagonal::Variances(variance_vector);
      gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
          0, poseEigToGtsamPose(current_frame_.pose_eig_), prior_noise));
      init_esti_.insert(current_keyframe_idx_,
                        poseEigToGtsamPose(current_frame_.pose_eig_));
      current_keyframe_idx_++;
      is_initialized_ = true;
    } else {
      //// 2. check if keyframe
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      if (checkIfKeyframe(current_frame_, keyframes_.back())) {
        // 2-2. if so, save
        {
          std::lock_guard<std::mutex> lock(keyframes_mutex_);
          keyframes_.push_back(current_frame_);
        }
        // 2-3. if so, add to graph
        auto variance_vector =
            (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
        gtsam::noiseModel::Diagonal::shared_ptr odom_noise =
            gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam::Pose3 pose_from = poseEigToGtsamPose(
            keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
        gtsam::Pose3 pose_to =
            poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
              current_keyframe_idx_ - 1, current_keyframe_idx_,
              pose_from.between(pose_to), odom_noise));
          init_esti_.insert(current_keyframe_idx_, pose_to);
        }
        current_keyframe_idx_++;

        //// 3. vis
        high_resolution_clock::time_point t3 = high_resolution_clock::now();
        {
          std::lock_guard<std::mutex> lock(vis_mutex_);
          updateOdomsAndPaths(current_frame_);
        }

        //// 4. optimize with graph
        high_resolution_clock::time_point t4 = high_resolution_clock::now();
        // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph,
        // init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
        {
          std::lock_guard<std::mutex> lock(graph_mutex_);
          isam_handler_->update(gtsam_graph_, init_esti_);
          isam_handler_->update();
          if (loop_added_flag_) // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
          {
            isam_handler_->update();
            isam_handler_->update();
            isam_handler_->update();
          }
          gtsam_graph_.resize(0);
          init_esti_.clear();
        }

        //// 5. handle corrected results
        // get corrected poses and reset odom delta (for realtime pose pub)
        high_resolution_clock::time_point t5 = high_resolution_clock::now();
        {
          std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
          corrected_esti_ = isam_handler_->calculateEstimate();
          last_corrected_pose_ = gtsamPoseToPoseEig(
              corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
          odom_delta_ = Eigen::Matrix4d::Identity();
        }
        // correct poses in keyframes
        if (loop_added_flag_) {
          std::lock_guard<std::mutex> lock(keyframes_mutex_);
          for (size_t i = 0; i < corrected_esti_.size(); ++i) {
            keyframes_[i].pose_corrected_eig_ =
                gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
          }
          loop_added_flag_ = false;
        }
        high_resolution_clock::time_point t6 = high_resolution_clock::now();

        RCLCPP_INFO(
            this->get_logger(),
            "real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, "
            "tot: %.1fms",
            duration_cast<microseconds>(t2 - t1).count() / 1e3,
            duration_cast<microseconds>(t3 - t2).count() / 1e3,
            duration_cast<microseconds>(t4 - t3).count() / 1e3,
            duration_cast<microseconds>(t5 - t4).count() / 1e3,
            duration_cast<microseconds>(t6 - t5).count() / 1e3,
            duration_cast<microseconds>(t6 - t1).count() / 1e3);
      }
    }
    return;
  }

  void saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg) {
    std::string save_dir = msg->data != "" ? msg->data : package_path_;

    // save scans as individual pcd files and poses in KITTI format
    // Delete the scans folder if it exists and create a new one
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_) {
      RCLCPP_INFO(
          this->get_logger(),
          "\033[32;1mScans are saved in %s, following the KITTI and TUM "
          "format\033[0m",
          scans_directory.c_str());
      if (fs::exists(seq_directory)) {
        fs::remove_all(seq_directory);
      }
      fs::create_directories(scans_directory);

      std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
      std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
      tum_pose_file << "#timestamp x y z qx qy qz qw\n";
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
          // Save the point cloud
          std::stringstream ss_;
          ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0')
              << i << ".pcd";
          RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
          pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

          // Save the pose in KITTI format
          const auto &pose_ = keyframes_[i].pose_corrected_eig_;
          kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " "
                          << pose_(0, 2) << " " << pose_(0, 3) << " "
                          << pose_(1, 0) << " " << pose_(1, 1) << " "
                          << pose_(1, 2) << " " << pose_(1, 3) << " "
                          << pose_(2, 0) << " " << pose_(2, 1) << " "
                          << pose_(2, 2) << " " << pose_(2, 3) << "\n";

          const auto &lidar_optim_pose_ =
              poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
          tum_pose_file << std::fixed << std::setprecision(8)
                        << keyframes_[i].timestamp_ << " "
                        << lidar_optim_pose_.pose.position.x << " "
                        << lidar_optim_pose_.pose.position.y << " "
                        << lidar_optim_pose_.pose.position.z << " "
                        << lidar_optim_pose_.pose.orientation.x << " "
                        << lidar_optim_pose_.pose.orientation.y << " "
                        << lidar_optim_pose_.pose.orientation.z << " "
                        << lidar_optim_pose_.pose.orientation.w << "\n";
        }
      }
      kitti_pose_file.close();
      tum_pose_file.close();
      RCLCPP_INFO(
          this->get_logger(),
          "\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }

    if (save_map_bag_) {
      rosbag2_cpp::Writer writer;
      rosbag2_storage::StorageOptions write_storage_options{};
      write_storage_options.uri = package_path_ + "/result.bag";
      write_storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options{};
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      writer.open(write_storage_options, converter_options);
      writer.create_topic(
          {"/keyframe_pcd", "sensor_msgs/msg/PointCloud2", "cdr", ""});
      writer.create_topic(
          {"/keyframe_pose", "geometry_msgs/msg/PoseStamped", "cdr", ""});
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
          rclcpp::Time time = fromSec(keyframes_[i].timestamp_);
          writer.write(pclToPclRos(keyframes_[i].pcd_, map_frame_),
                       "/keyframe_pcd", time);
          writer.write(poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_),
                       "/keyframe_pose", time);
        }
      }
      writer.close();
      RCLCPP_INFO(this->get_logger(),
                  "\033[36;1mResult saved in .bag format!!!\033[0m");
    }

    if (save_map_pcd_) {
      pcl::PointCloud<PointType>::Ptr corrected_map(
          new pcl::PointCloud<PointType>());
      corrected_map->reserve(keyframes_[0].pcd_.size() *
                             keyframes_.size()); // it's an approximated size
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
          *corrected_map += transformPcd(keyframes_[i].pcd_,
                                         keyframes_[i].pose_corrected_eig_);
        }
      }
      const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
      pcl::io::savePCDFileASCII<PointType>(
          seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
      RCLCPP_INFO(
          this->get_logger(),
          "\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
    }
  }

  void loopTimerFunc() {
    auto &latest_keyframe = keyframes_.back();
    if (!is_initialized_ || keyframes_.empty() || latest_keyframe.processed_) {
      return;
    }
    latest_keyframe.processed_ = true;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    const int closest_keyframe_idx =
        loop_closure_->fetchClosestKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0) {
      return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(
        latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_) {
      RCLCPP_INFO(this->get_logger(),
                  "\033[1;32mLoop closure accepted. Score: %.3f\033[0m",
                  reg_output.score_);
      const auto &score = reg_output.score_;
      gtsam::Pose3 pose_from = poseEigToGtsamPose(
          reg_output.pose_between_eig_ *
          latest_keyframe
              .pose_corrected_eig_); // IMPORTANT: take care of the order
      gtsam::Pose3 pose_to = poseEigToGtsamPose(
          keyframes_[closest_keyframe_idx].pose_corrected_eig_);
      auto variance_vector =
          (gtsam::Vector(6) << score, score, score, score, score, score)
              .finished();
      gtsam::noiseModel::Diagonal::shared_ptr loop_noise =
          gtsam::noiseModel::Diagonal::Variances(variance_vector);
      {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            latest_keyframe.idx_, closest_keyframe_idx,
            pose_from.between(pose_to), loop_noise));
      }
      loop_idx_pairs_.push_back(
          {latest_keyframe.idx_, closest_keyframe_idx}); // for vis
      loop_added_flag_vis_ = true;
      loop_added_flag_ = true;
    } else {
      RCLCPP_WARN(this->get_logger(), "Loop closure rejected. Score: %.3f",
                  reg_output.score_);
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    debug_src_pub_->publish(
        pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(
        pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(
        pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
    debug_coarse_aligned_pub_->publish(
        pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

    RCLCPP_INFO(this->get_logger(), "loop: %.1f",
                duration_cast<microseconds>(t2 - t1).count() / 1e3);
    return;
  }

  void visTimerFunc() {
    if (!is_initialized_) {
      return;
    }

    high_resolution_clock::time_point tv1 = high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
      gtsam::Values corrected_esti_copied;
      pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
      PathT corrected_path;
      {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        corrected_esti_copied = corrected_esti_;
      }
      // correct pose and path
      for (size_t i = 0; i < corrected_esti_copied.size(); ++i) {
        gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
        corrected_odoms.points.emplace_back(pose_.translation().x(),
                                            pose_.translation().y(),
                                            pose_.translation().z());
        corrected_path.poses.push_back(
            gtsamPoseToPoseStamped(pose_, map_frame_));
      }
      // update vis of loop constraints
      if (!loop_idx_pairs_.empty()) {
        loop_detection_pub_->publish(getLoopMarkers(corrected_esti_copied));
      }
      // update with corrected data
      {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        corrected_odoms_ = corrected_odoms;
        corrected_path_.poses = corrected_path.poses;
      }
      loop_added_flag_vis_ = false;
    }
    //// 2. publish odoms, paths
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      odom_pub_->publish(pclToPclRos(odoms_, map_frame_));
      path_pub_->publish(odom_path_);
      corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
      corrected_path_pub_->publish(corrected_path_);
    }

    //// 3. global map
    if (global_map_vis_switch_ &&
        corrected_pcd_map_pub_->get_subscription_count() >
            0) // save time, only once
    {
      pcl::PointCloud<PointType>::Ptr corrected_map(
          new pcl::PointCloud<PointType>());
      corrected_map->reserve(keyframes_[0].pcd_.size() *
                             keyframes_.size()); // it's an approximated size
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i) {
          *corrected_map += transformPcd(keyframes_[i].pcd_,
                                         keyframes_[i].pose_corrected_eig_);
        }
      }
      const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
      corrected_pcd_map_pub_->publish(pclToPclRos(*voxelized_map, map_frame_));
      global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ &&
        corrected_pcd_map_pub_->get_subscription_count() == 0) {
      global_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point tv2 = high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "vis: %.1fms",
                duration_cast<microseconds>(tv2 - tv1).count() / 1e3);
    return;
  }
};