# Copyright 2025 Lihan Chen
# Copyright 2024 Hongbiao Zhu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Original work based on sensor_scan_generation package by Hongbiao Zhu.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrain_analysis_node",
        name="terrain_analysis",
        output="screen",
        namespace="",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        parameters=[
            {"sensor_frame": "front_mid360"},
            {"scan_voxel_size": 0.05},
            {"decay_time": 1.0},
            {"no_decay_dis": 1.75},
            {"clearing_dis": 8.0},
            {"use_sorting": True},
            {"quantile_z": 0.5},
            {"consider_drop": False},
            {"limit_ground_lift": False},
            {"max_ground_lift": 0.15},
            {"clear_dy_obs": True},
            {"min_dy_obs_dis": 0.3},
            {"min_dy_obs_angle": 0.0},
            {"min_dy_obs_rel_z": -0.3},
            {"abs_dy_obs_rel_z_thre": 0.2},
            {"min_dy_obs_vfov": -28.0},
            {"max_dy_obs_vfov": 33.0},
            {"min_dy_obs_point_num": 1},
            {"no_data_obstacle": False},
            {"no_data_block_skip_num": 0},
            {"min_block_point_num": 10},
            {"vehicle_height": 0.5},
            {"voxel_point_update_thre": 100},
            {"voxel_time_update_thre": 2.0},
            {"min_rel_z": -1.5},
            {"max_rel_z": 0.3},
            {"dis_ratio_z": 0.2},
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(start_terrain_analysis_cmd)

    return ld
