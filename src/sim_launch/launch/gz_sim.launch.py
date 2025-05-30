# type: ignore
# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Launch Gazebo Sim with command line arguments."""

import os

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import (
    PACKAGE_MANIFEST_FILENAME,
    InvalidPackage,
    parse_package,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ros2pkg.api import get_package_names

# Copied from https://github.com/ros-simulation/gazebo_ros_pkgs/blob/79fd94c6da76781a91499bc0f54b70560b90a9d2/gazebo_ros/scripts/gazebo_ros_paths.py
"""
Search for model, plugin and media paths exported by packages.

e.g.  <export>
          <gazebo_ros gazebo_model_path="${prefix}/../"/>
          <gazebo_ros gazebo_media_path="${prefix}/../"/>
      </export>
${prefix} is replaced by package's share directory in install.

Thus the required directory needs to be installed from CMakeLists.txt
e.g.  install(DIRECTORY models
          DESTINATION share/${PROJECT_NAME})
"""

def launch_gz_sim(context, *args, **kwargs):
    gz_args = LaunchConfiguration('gz_args').perform(context)
    gz_version = LaunchConfiguration('gz_version').perform(context)
    world_path = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    env = {
        # プラグインパスは明示的に必要な場合のみ環境変数で指定すること
        # "GZ_SIM_SYSTEM_PLUGIN_PATH": "",
        # "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": "",
        "GZ_SIM_RESOURCE_PATH": os.pathsep.join([
            os.environ.get("GZ_SIM_RESOURCE_PATH", default=""),
            # model_paths,  # ← 削除
        ]),
        "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join([
            os.environ.get("IGN_GAZEBO_RESOURCE_PATH", default=""),
            # model_paths,  # ← 削除
        ]),
    }
    exec_args = gz_args
    if world_path:
        exec_args += f' --world {world_path}'
    if headless:
        actions = []
        actions.append(SetEnvironmentVariable('DISPLAY', ''))
        # DISPLAY未設定時も必ずヘッドレス
        if not os.environ.get('DISPLAY') and '--headless-rendering' not in exec_args:
            exec_args += ' --headless-rendering'
        cmd = f"gz sim {exec_args} --force-version {gz_version}"
        actions.append(ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env=env,
            shell=True
        ))
        return actions
    else:
        return []

def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value='--headless-rendering',
            description='Arguments to be passed to Gazebo Sim',
        ),
        DeclareLaunchArgument('gz_version', default_value='6', description="Gazebo Sim's major version"),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                get_package_share_directory('sim_launch'), 'launch', 'empty_custom.sdf'),
            description='Path to the SDF world file to load',
        ),
        DeclareLaunchArgument(
            'ign_args', default_value='',
            description='Deprecated: Arguments to be passed to Gazebo Sim'
        ),
        DeclareLaunchArgument(
            'ign_version', default_value='',
            description="Deprecated: Gazebo Sim's major version"
        ),
        DeclareLaunchArgument('debugger', default_value='false', description='Run in Debugger'),
        DeclareLaunchArgument('on_exit_shutdown', default_value='false', description='Shutdown on gz-sim exit'),
        DeclareLaunchArgument('headless', default_value='false', description='Run in headless mode'),
        DeclareLaunchArgument('record', default_value='false', description='Enable gz record (simulation recording)'),
        ExecuteProcess(
            cmd=['gz', 'record', '-o', '/logs/sim_record'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('record')),
        ),
        OpaqueFunction(function=launch_gz_sim)
    ])
    return ld
