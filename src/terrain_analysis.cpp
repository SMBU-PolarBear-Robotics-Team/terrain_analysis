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

#include "terrain_analysis/terrain_analysis.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace terrain_analysis
{

TerrainAnalysisNode::TerrainAnalysisNode(const rclcpp::NodeOptions & options)
: Node("terrain_analysis", options)
{
  declare_parameter<std::string>("sensor_frame", sensor_frame_);
  declare_parameter<double>("scan_voxel_size", scan_voxel_size_);
  declare_parameter<double>("decay_time", decay_time_);
  declare_parameter<double>("no_decay_dis", no_decay_dis_);
  declare_parameter<double>("clearing_dis", clearing_dis_);
  declare_parameter<bool>("use_sorting", use_sorting_);
  declare_parameter<double>("quantile_z", quantile_z_);
  declare_parameter<bool>("consider_drop", consider_drop_);
  declare_parameter<bool>("limit_ground_lift", limit_ground_lift_);
  declare_parameter<double>("max_ground_lift", max_ground_lift_);
  declare_parameter<bool>("clear_dy_obs", clear_dy_obs_);
  declare_parameter<double>("min_dy_obs_dis", min_dy_obs_dis_);
  declare_parameter<double>("min_dy_obs_angle", min_dy_obs_angle_);
  declare_parameter<double>("min_dy_obs_rel_z", min_dy_obs_rel_z_);
  declare_parameter<double>("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
  declare_parameter<double>("min_dy_obs_vfov", min_dy_obs_vfov_);
  declare_parameter<double>("max_dy_obs_vfov", max_dy_obs_vfov_);
  declare_parameter<int>("min_dy_obs_point_num", min_dy_obs_point_num_);
  declare_parameter<bool>("no_data_obstacle", no_data_obstacle_);
  declare_parameter<int>("no_data_block_skip_num", no_data_block_skip_num_);
  declare_parameter<int>("min_block_point_num", min_block_point_num_);
  declare_parameter<double>("vehicle_height", vehicle_height_);
  declare_parameter<int>("voxel_point_update_thre", voxel_point_update_thre_);
  declare_parameter<double>("voxel_time_update_thre", voxel_time_update_thre_);
  declare_parameter<double>("min_rel_z", min_rel_z_);
  declare_parameter<double>("max_rel_z", max_rel_z_);
  declare_parameter<double>("dis_ratio_z", dis_ratio_z_);

  get_parameter("sensor_frame", sensor_frame_);
  get_parameter("scan_voxel_size", scan_voxel_size_);
  get_parameter("decay_time", decay_time_);
  get_parameter("no_decay_dis", no_decay_dis_);
  get_parameter("clearing_dis", clearing_dis_);
  get_parameter("use_sorting", use_sorting_);
  get_parameter("quantile_z", quantile_z_);
  get_parameter("consider_drop", consider_drop_);
  get_parameter("limit_ground_lift", limit_ground_lift_);
  get_parameter("max_ground_lift", max_ground_lift_);
  get_parameter("clear_dy_obs", clear_dy_obs_);
  get_parameter("min_dy_obs_dis", min_dy_obs_dis_);
  get_parameter("minDyObAngle", min_dy_obs_angle_);
  get_parameter("min_dy_obs_rel_z", min_dy_obs_rel_z_);
  get_parameter("abs_dy_obs_rel_z_thre", abs_dy_obs_rel_z_thre_);
  get_parameter("min_dy_obs_vfov", min_dy_obs_vfov_);
  get_parameter("max_dy_obs_vfov", max_dy_obs_vfov_);
  get_parameter("min_dy_obs_point_num", min_dy_obs_point_num_);
  get_parameter("no_data_obstacle", no_data_obstacle_);
  get_parameter("no_data_block_skip_num", no_data_block_skip_num_);
  get_parameter("min_block_point_num", min_block_point_num_);
  get_parameter("vehicle_height", vehicle_height_);
  get_parameter("voxel_point_update_thre", voxel_point_update_thre_);
  get_parameter("voxel_time_update_thre", voxel_time_update_thre_);
  get_parameter("min_rel_z", min_rel_z_);
  get_parameter("max_rel_z", max_rel_z_);
  get_parameter("dis_ratio_z", dis_ratio_z_);

  laser_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_crop_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_dwz_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_elev_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (auto & i : terrain_voxel_cloud_) {
    i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "lidar_odometry", 5,
    std::bind(&TerrainAnalysisNode::odometryHandler, this, std::placeholders::_1));
  laser_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 5,
    std::bind(&TerrainAnalysisNode::laserCloudHandler, this, std::placeholders::_1));
  joystick_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 5, std::bind(&TerrainAnalysisNode::joystickHandler, this, std::placeholders::_1));
  clearing_sub_ = create_subscription<example_interfaces::msg::Float32>(
    "map_clearing", 5,
    std::bind(&TerrainAnalysisNode::clearingHandler, this, std::placeholders::_1));
  terrain_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);

  for (auto & i : terrain_voxel_cloud_) {
    i = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }

  down_size_filter_.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
}

void TerrainAnalysisNode::odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  odom_ = *odom;
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w))
    .getRPY(roll, pitch, yaw);

  sin_vehicle_roll_ = sin(roll);
  cos_vehicle_roll_ = cos(roll);
  sin_vehicle_pitch_ = sin(pitch);
  cos_vehicle_pitch_ = cos(pitch);
  sin_vehicle_yaw_ = sin(yaw);
  cos_vehicle_yaw_ = cos(yaw);

  if (no_data_inited_ == 0) {
    vehicle_x_rec_ = odom_.pose.pose.position.x;
    vehicle_y_rec_ = odom_.pose.pose.position.y;
    no_data_inited_ = 1;
  }
  if (no_data_inited_ == 1) {
    float dis = sqrt(
      (odom_.pose.pose.position.x - vehicle_x_rec_) *
        (odom_.pose.pose.position.x - vehicle_x_rec_) +
      (odom_.pose.pose.position.y - vehicle_y_rec_) *
        (odom_.pose.pose.position.y - vehicle_y_rec_));
    if (dis >= no_decay_dis_) no_data_inited_ = 2;
  }
}

void TerrainAnalysisNode::laserCloudHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud)
{
  laser_cloud_time_ = rclcpp::Time(laserCloud->header.stamp).seconds();
  if (!system_inited_) {
    system_init_time_ = laser_cloud_time_;
    system_inited_ = true;
  }

  laser_cloud_->clear();
  pcl::fromROSMsg(*laserCloud, *laser_cloud_);

  pcl::PointXYZI point;
  laser_cloud_crop_->clear();
  int laser_cloud_size = laser_cloud_->points.size();
  for (int i = 0; i < laser_cloud_size; i++) {
    point = laser_cloud_->points[i];

    float point_x = point.x;
    float point_y = point.y;
    float point_z = point.z;

    float dis = sqrt(
      (point_x - odom_.pose.pose.position.x) * (point_x - odom_.pose.pose.position.x) +
      (point_y - odom_.pose.pose.position.y) * (point_y - odom_.pose.pose.position.y));
    if (
      point_z - odom_.pose.pose.position.z > min_rel_z_ - dis_ratio_z_ * dis &&
      point_z - odom_.pose.pose.position.z < max_rel_z_ + dis_ratio_z_ * dis &&
      dis < terrain_voxel_size_ * (terrain_voxel_half_width_ + 1)) {
      point.x = point_x;
      point.y = point_y;
      point.z = point_z;
      point.intensity = laser_cloud_time_ - system_init_time_;
      laser_cloud_crop_->push_back(point);
    }
  }

  processLaserCloud();
}

void TerrainAnalysisNode::joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->buttons[5] > 0.5) {
    no_data_inited_ = 0;
    clearing_cloud_ = true;
  }
}

void TerrainAnalysisNode::clearingHandler(
  const example_interfaces::msg::Float32::ConstSharedPtr dis)
{
  no_data_inited_ = 0;
  clearing_dis_ = dis->data;
  clearing_cloud_ = true;
}

void TerrainAnalysisNode::processLaserCloud()
{
  // terrain voxel roll over
  float terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  float terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;

  while (odom_.pose.pose.position.x - terrain_voxel_cen_x < -terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ind_y++) {
      auto terrain_voxel_cloud_ptr =
        terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y];
      for (int ind_x = terrain_voxel_width_ - 1; ind_x >= 1; ind_x--) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * (ind_x - 1) + ind_y];
      }
      terrain_voxel_cloud_[ind_y] = terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[ind_y]->clear();
    }
    terrain_voxel_shift_x_--;
    terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (odom_.pose.pose.position.x - terrain_voxel_cen_x > terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ind_y++) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind_y];
      for (int ind_x = 0; ind_x < terrain_voxel_width_ - 1; ind_x++) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * (ind_x + 1) + ind_y];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y] =
        terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y]->clear();
    }
    terrain_voxel_shift_x_++;
    terrain_voxel_cen_x = terrain_voxel_size_ * terrain_voxel_shift_x_;
  }

  while (odom_.pose.pose.position.y - terrain_voxel_cen_y < -terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ind_x++) {
      auto terrain_voxel_cloud_ptr =
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)];
      for (int ind_y = terrain_voxel_width_ - 1; ind_y >= 1; ind_y--) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (ind_y - 1)];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x] = terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x]->clear();
    }
    terrain_voxel_shift_y_--;
    terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  while (odom_.pose.pose.position.y - terrain_voxel_cen_y > terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ind_x++) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[terrain_voxel_width_ * ind_x];
      for (int ind_y = 0; ind_y < terrain_voxel_width_ - 1; ind_y++) {
        terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y] =
          terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (ind_y + 1)];
      }
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)] =
        terrain_voxel_cloud_ptr;
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)]->clear();
    }
    terrain_voxel_shift_y_++;
    terrain_voxel_cen_y = terrain_voxel_size_ * terrain_voxel_shift_y_;
  }

  // stack registered laser scans
  pcl::PointXYZI point;
  int laser_cloud_crop_size = laser_cloud_crop_->points.size();
  for (int i = 0; i < laser_cloud_crop_size; i++) {
    point = laser_cloud_crop_->points[i];

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width_;

    if (point.x - odom_.pose.pose.position.x + terrain_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + terrain_voxel_size_ / 2 < 0) ind_y--;

    if (ind_x >= 0 && ind_x < terrain_voxel_width_ && ind_y >= 0 && ind_y < terrain_voxel_width_) {
      terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y]->push_back(point);
      terrain_voxel_update_num_[terrain_voxel_width_ * ind_x + ind_y]++;
    }
  }

  for (int ind = 0; ind < terrain_voxel_num_; ind++) {
    if (
      terrain_voxel_update_num_[ind] >= voxel_point_update_thre_ ||
      laser_cloud_time_ - system_init_time_ - terrain_voxel_update_time_[ind] >=
        voxel_time_update_thre_ ||
      clearing_cloud_) {
      auto terrain_voxel_cloud_ptr = terrain_voxel_cloud_[ind];

      laser_cloud_dwz_->clear();
      down_size_filter_.setInputCloud(terrain_voxel_cloud_ptr);
      down_size_filter_.filter(*laser_cloud_dwz_);

      terrain_voxel_cloud_ptr->clear();
      int laser_cloud_dwz_size = laser_cloud_dwz_->points.size();
      for (int i = 0; i < laser_cloud_dwz_size; i++) {
        point = laser_cloud_dwz_->points[i];
        float dis = sqrt(
          (point.x - odom_.pose.pose.position.x) * (point.x - odom_.pose.pose.position.x) +
          (point.y - odom_.pose.pose.position.y) * (point.y - odom_.pose.pose.position.y));
        if (
          point.z - odom_.pose.pose.position.z > min_rel_z_ - dis_ratio_z_ * dis &&
          point.z - odom_.pose.pose.position.z < max_rel_z_ + dis_ratio_z_ * dis &&
          (laser_cloud_time_ - system_init_time_ - point.intensity < decay_time_ ||
           dis < no_decay_dis_) &&
          (dis >= clearing_dis_ || !clearing_cloud_)) {
          terrain_voxel_cloud_ptr->push_back(point);
        }
      }

      terrain_voxel_update_num_[ind] = 0;
      terrain_voxel_update_time_[ind] = laser_cloud_time_ - system_init_time_;
    }
  }

  terrain_cloud_->clear();
  for (int ind_x = terrain_voxel_half_width_ - 5; ind_x <= terrain_voxel_half_width_ + 5; ind_x++) {
    for (int ind_y = terrain_voxel_half_width_ - 5; ind_y <= terrain_voxel_half_width_ + 5;
         ind_y++) {
      *terrain_cloud_ += *terrain_voxel_cloud_[terrain_voxel_width_ * ind_x + ind_y];
    }
  }

  // estimate ground and compute elevation for each point
  for (int i = 0; i < planar_voxel_num_; i++) {
    planar_voxel_elev_[i] = 0;
    planar_voxel_edge_[i] = 0;
    planar_voxel_dy_obs_[i] = 0;
    planar_point_elev_[i].clear();
  }

  int terrain_cloud_size = terrain_cloud_->points.size();
  for (int i = 0; i < terrain_cloud_size; i++) {
    point = terrain_cloud_->points[i];

    int ind_x =
      static_cast<int>(
        (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;
    int ind_y =
      static_cast<int>(
        (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
      planar_voxel_half_width_;

    if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

    if (
      point.z - odom_.pose.pose.position.z > min_rel_z_ &&
      point.z - odom_.pose.pose.position.z < max_rel_z_) {
      for (int d_x = -1; d_x <= 1; d_x++) {
        for (int d_y = -1; d_y <= 1; d_y++) {
          if (
            ind_x + d_x >= 0 && ind_x + d_x < planar_voxel_width_ && ind_y + d_y >= 0 &&
            ind_y + d_y < planar_voxel_width_) {
            planar_point_elev_[planar_voxel_width_ * (ind_x + d_x) + ind_y + d_y].push_back(
              point.z);
          }
        }
      }
    }

    if (clear_dy_obs_) {
      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        float point_x1 = point.x - odom_.pose.pose.position.x;
        float point_y1 = point.y - odom_.pose.pose.position.y;
        float point_z1 = point.z - odom_.pose.pose.position.z;

        float dis1 = sqrt(point_x1 * point_x1 + point_y1 * point_y1);
        if (dis1 > min_dy_obs_dis_) {
          float angle1 = atan2(point_z1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
          if (angle1 > min_dy_obs_angle_) {
            float point_x2 = point_x1 * cos_vehicle_yaw_ + point_y1 * sin_vehicle_yaw_;
            float point_y2 = -point_x1 * sin_vehicle_yaw_ + point_y1 * cos_vehicle_yaw_;
            float point_z2 = point_z1;

            float point_x3 = point_x2 * cos_vehicle_pitch_ - point_z2 * sin_vehicle_pitch_;
            float point_y3 = point_y2;
            float point_z3 = point_x2 * sin_vehicle_pitch_ + point_z2 * cos_vehicle_pitch_;

            float point_x4 = point_x3;
            float point_y4 = point_y3 * cos_vehicle_roll_ + point_z3 * sin_vehicle_roll_;
            float point_z4 = -point_y3 * sin_vehicle_roll_ + point_z3 * cos_vehicle_roll_;

            float dis4 = sqrt(point_x4 * point_x4 + point_y4 * point_y4);
            float angle4 = atan2(point_z4, dis4) * 180.0 / M_PI;
            if (
              (angle4 > min_dy_obs_vfov_ && angle4 < max_dy_obs_vfov_) ||
              fabs(point_z4) < abs_dy_obs_rel_z_thre_) {
              planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y]++;
            }
          }
        } else {
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] += min_dy_obs_point_num_;
        }
      }
    }
  }

  if (clear_dy_obs_) {
    for (int i = 0; i < laser_cloud_crop_size; i++) {
      point = laser_cloud_crop_->points[i];

      int ind_x =
        static_cast<int>(
          (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;
      int ind_y =
        static_cast<int>(
          (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;

      if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
      if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        float point_x1 = point.x - odom_.pose.pose.position.x;
        float point_y1 = point.y - odom_.pose.pose.position.y;
        float point_z1 = point.z - odom_.pose.pose.position.z;

        float dis1 = sqrt(point_x1 * point_x1 + point_y1 * point_y1);
        float angle1 = atan2(point_z1 - min_dy_obs_rel_z_, dis1) * 180.0 / M_PI;
        if (angle1 > min_dy_obs_angle_) {
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] = 0;
        }
      }
    }
  }

  if (use_sorting_) {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size > 0) {
        std::sort(planar_point_elev_[i].begin(), planar_point_elev_[i].end());

        int quantile_id = static_cast<int>(quantile_z_ * planar_point_elev_size);
        if (quantile_id < 0)
          quantile_id = 0;
        else if (quantile_id >= planar_point_elev_size)
          quantile_id = planar_point_elev_size - 1;

        if (
          planar_point_elev_[i][quantile_id] > planar_point_elev_[i][0] + max_ground_lift_ &&
          limit_ground_lift_) {
          planar_voxel_elev_[i] = planar_point_elev_[i][0] + max_ground_lift_;
        } else {
          planar_voxel_elev_[i] = planar_point_elev_[i][quantile_id];
        }
      }
    }
  } else {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size > 0) {
        float min_z = 1000.0;
        int min_id = -1;
        for (int j = 0; j < planar_point_elev_size; j++) {
          if (planar_point_elev_[i][j] < min_z) {
            min_z = planar_point_elev_[i][j];
            min_id = j;
          }
        }

        if (min_id != -1) {
          planar_voxel_elev_[i] = planar_point_elev_[i][min_id];
        }
      }
    }
  }

  terrain_cloud_elev_->clear();
  int terrain_cloud_elev_size = 0;
  for (int i = 0; i < terrain_cloud_size; i++) {
    point = terrain_cloud_->points[i];
    if (
      point.z - odom_.pose.pose.position.z > min_rel_z_ &&
      point.z - odom_.pose.pose.position.z < max_rel_z_) {
      int ind_x =
        static_cast<int>(
          (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;
      int ind_y =
        static_cast<int>(
          (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
        planar_voxel_half_width_;

      if (point.x - odom_.pose.pose.position.x + planar_voxel_size_ / 2 < 0) ind_x--;
      if (point.y - odom_.pose.pose.position.y + planar_voxel_size_ / 2 < 0) ind_y--;

      if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
        if (
          planar_voxel_dy_obs_[planar_voxel_width_ * ind_x + ind_y] < min_dy_obs_point_num_ ||
          !clear_dy_obs_) {
          float dis_z = point.z - planar_voxel_elev_[planar_voxel_width_ * ind_x + ind_y];
          if (consider_drop_) dis_z = fabs(dis_z);
          int planar_point_elev_size =
            planar_point_elev_[planar_voxel_width_ * ind_x + ind_y].size();
          if (
            dis_z >= 0 && dis_z < vehicle_height_ &&
            planar_point_elev_size >= min_block_point_num_) {
            terrain_cloud_elev_->push_back(point);
            terrain_cloud_elev_->points[terrain_cloud_elev_size].intensity = dis_z;

            terrain_cloud_elev_size++;
          }
        }
      }
    }
  }

  if (no_data_obstacle_ && no_data_inited_ == 2) {
    for (int i = 0; i < planar_voxel_num_; i++) {
      int planar_point_elev_size = planar_point_elev_[i].size();
      if (planar_point_elev_size < min_block_point_num_) {
        planar_voxel_edge_[i] = 1;
      }
    }

    for (int no_data_block_skip_count = 0; no_data_block_skip_count < no_data_block_skip_num_;
         no_data_block_skip_count++) {
      for (int i = 0; i < planar_voxel_num_; i++) {
        if (planar_voxel_edge_[i] >= 1) {
          int ind_x = static_cast<int>(i / planar_voxel_width_);
          int ind_y = i % planar_voxel_width_;
          bool edge_voxel = false;
          for (int d_x = -1; d_x <= 1; d_x++) {
            for (int d_y = -1; d_y <= 1; d_y++) {
              if (
                ind_x + d_x >= 0 && ind_x + d_x < planar_voxel_width_ && ind_y + d_y >= 0 &&
                ind_y + d_y < planar_voxel_width_) {
                if (
                  planar_voxel_edge_[planar_voxel_width_ * (ind_x + d_x) + ind_y + d_y] <
                  planar_voxel_edge_[i]) {
                  edge_voxel = true;
                }
              }
            }
          }

          if (!edge_voxel) planar_voxel_edge_[i]++;
        }
      }
    }

    for (int i = 0; i < planar_voxel_num_; i++) {
      if (planar_voxel_edge_[i] > no_data_block_skip_num_) {
        int ind_x = static_cast<int>(i / planar_voxel_width_);
        int ind_y = i % planar_voxel_width_;

        point.x =
          planar_voxel_size_ * (ind_x - planar_voxel_half_width_) + odom_.pose.pose.position.x;
        point.y =
          planar_voxel_size_ * (ind_y - planar_voxel_half_width_) + odom_.pose.pose.position.y;
        point.z = odom_.pose.pose.position.z;
        point.intensity = vehicle_height_;

        point.x -= planar_voxel_size_ / 4.0;
        point.y -= planar_voxel_size_ / 4.0;
        terrain_cloud_elev_->push_back(point);

        point.x += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.y += planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);

        point.x -= planar_voxel_size_ / 2.0;
        terrain_cloud_elev_->push_back(point);
      }
    }
  }

  clearing_cloud_ = false;

  // Publish points with elevation
  sensor_msgs::msg::PointCloud2 terrain_cloud;
  pcl::toROSMsg(*terrain_cloud_elev_, terrain_cloud);
  terrain_cloud.header.stamp = rclcpp::Time(static_cast<uint64_t>(laser_cloud_time_ * 1e9));
  terrain_cloud.header.frame_id = "odom";

  // Transform point cloud to lidar frame
  sensor_msgs::msg::PointCloud2 terrain_cloud_lidar;
  tf2::Transform tf_odom_to_lidar;
  tf2::fromMsg(odom_.pose.pose, tf_odom_to_lidar);
  pcl_ros::transformPointCloud(
    sensor_frame_, tf_odom_to_lidar.inverse(), terrain_cloud, terrain_cloud_lidar);

  // Publish the transformed point cloud
  terrain_map_pub_->publish(terrain_cloud_lidar);
}
}  // namespace terrain_analysis

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis::TerrainAnalysisNode)
