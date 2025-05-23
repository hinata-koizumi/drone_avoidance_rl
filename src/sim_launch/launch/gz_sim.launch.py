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
from os import environ

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage
from catkin_pkg.package import PACKAGE_MANIFEST_FILENAME
from catkin_pkg.package import parse_package
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
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

class GazeboRosPaths:
    @staticmethod
    def get_paths():
        gazebo_model_path = []
        gazebo_plugin_path = []
        gazebo_media_path = []
        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)
            package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
            if os.path.isfile(package_file_path):
                try:
                    package = parse_package(package_file_path)
                except InvalidPackage:
                    continue
                for export in package.exports:
                    if export.tagname == 'gazebo_ros':
                        if 'gazebo_model_path' in export.attributes:
                            xml_path = export.attributes['gazebo_model_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_model_path.append(xml_path)
                        if 'plugin_path' in export.attributes:
                            xml_path = export.attributes['plugin_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_plugin_path.append(xml_path)
                        if 'gazebo_media_path' in export.attributes:
                            xml_path = export.attributes['gazebo_media_path']
                            xml_path = xml_path.replace('${prefix}', package_share_path)
                            gazebo_media_path.append(xml_path)
        gazebo_model_path = os.pathsep.join(gazebo_model_path + gazebo_media_path)
        gazebo_plugin_path = os.pathsep.join(gazebo_plugin_path)
        return gazebo_model_path, gazebo_plugin_path

def generate_launch_description():
    # Launch arguments
    gz_args = LaunchConfiguration('gz_args')
    gz_version = LaunchConfiguration('gz_version')
    model_paths, plugin_paths = GazeboRosPaths.get_paths()
    env = {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join([
            environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
            environ.get("LD_LIBRARY_PATH", default=""),
            plugin_paths,
        ]),
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": os.pathsep.join([
            environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
            environ.get("LD_LIBRARY_PATH", default=""),
            plugin_paths,
        ]),
        "GZ_SIM_RESOURCE_PATH": os.pathsep.join([
            environ.get("GZ_SIM_RESOURCE_PATH", default=""),
            model_paths,
        ]),
        "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join([
            environ.get("IGN_GAZEBO_RESOURCE_PATH", default=""),
            model_paths,
        ]),
    }
    # Compose the command
    exec_args = gz_args
    exec = 'gz sim'
    debug_prefix = None
    on_exit = None
    # Declare launch arguments
    ld = LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='--headless-rendering', description='Arguments to be passed to Gazebo Sim'),
        DeclareLaunchArgument('gz_version', default_value='6', description="Gazebo Sim's major version"),
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
    ])
    # Add the main process
    ld.add_action(ExecuteProcess(
        cmd=[exec, exec_args, '--force-version', gz_version],
        output='screen',
        additional_env=env,
        shell=True,
        prefix=debug_prefix,
        on_exit=on_exit
    ))
    return ld
