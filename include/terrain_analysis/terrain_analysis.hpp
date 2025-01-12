// Copyright 2025 Lihan Chen
// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#ifndef TERRAIN_ANALYSIS__TERRAIN_ANALYSIS_HPP_
#define TERRAIN_ANALYSIS__TERRAIN_ANALYSIS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "example_interfaces/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace terrain_analysis
{

class TerrainAnalysisNode : public rclcpp::Node
{
public:
  explicit TerrainAnalysisNode(const rclcpp::NodeOptions & options);

private:
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2);
  void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy);
  void clearingHandler(const example_interfaces::msg::Float32::ConstSharedPtr dis);
  void processLaserCloud();

  std::string sensor_frame_;
  double scan_voxel_size_;
  double decay_time_;
  double no_decay_dis_;
  double clearing_dis_;
  bool clearing_cloud_;
  bool use_sorting_;
  double quantile_z_;
  bool consider_drop_;
  bool limit_ground_lift_;
  double max_ground_lift_;
  bool clear_dy_obs_;
  double min_dy_obs_dis_;
  double min_dy_obs_angle_;
  double min_dy_obs_rel_z_;
  double abs_dy_obs_rel_z_thre_;
  double min_dy_obs_vfov_;
  double max_dy_obs_vfov_;
  int min_dy_obs_point_num_;
  bool no_data_obstacle_;
  int no_data_block_skip_num_;
  int min_block_point_num_;
  double vehicle_height_;
  int voxel_point_update_thre_;
  double voxel_time_update_thre_;
  double min_rel_z_;
  double max_rel_z_;
  double dis_ratio_z_;

  // terrain voxel parameters
  int terrain_voxel_shift_x_ = 0;
  int terrain_voxel_shift_y_ = 0;
  const float terrain_voxel_size_ = 1.0;
  const int terrain_voxel_width_ = 21;
  const int terrain_voxel_half_width_ = (terrain_voxel_width_ - 1) / 2;
  const int terrain_voxel_num_ = terrain_voxel_width_ * terrain_voxel_width_;

  // planar voxel parameters
  const float planar_voxel_size_ = 0.2;
  const int planar_voxel_width_ = 51;
  const int planar_voxel_half_width_ = (planar_voxel_width_ - 1) / 2;
  const int planar_voxel_num_ = planar_voxel_width_ * planar_voxel_width_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> laser_cloud_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> laser_cloud_crop_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> laser_cloud_dwz_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> terrain_cloud_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> terrain_cloud_elev_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> terrain_voxel_cloud_[441];

  int terrain_voxel_update_num_[441];
  float terrain_voxel_update_time_[441];
  float planar_voxel_elev_[2601];
  int planar_voxel_edge_[2601];
  int planar_voxel_dy_obs_[2601];
  std::vector<float> planar_point_elev_[2601];

  double laser_cloud_time_;

  double system_init_time_;
  bool system_inited_;
  int no_data_inited_;

  float vehicle_x_rec_, vehicle_y_rec_;

  float sin_vehicle_roll_, cos_vehicle_roll_;
  float sin_vehicle_pitch_, cos_vehicle_pitch_;
  float sin_vehicle_yaw_, cos_vehicle_yaw_;

  nav_msgs::msg::Odometry odom_;
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr clearing_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_pub_;
};

}  // namespace terrain_analysis

#endif  // TERRAIN_ANALYSIS__TERRAIN_ANALYSIS_HPP_
