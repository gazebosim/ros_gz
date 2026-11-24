# Copyright 2024 Open Source Robotics Foundation, Inc.
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

import os
from typing import Dict

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package


class GazeboRosPaths:

    @staticmethod
    def get_paths():
        gazebo_model_path = []
        gazebo_plugin_path = []
        gazebo_media_path = []

        for package_name in get_packages_with_prefixes().keys():
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

    @staticmethod
    def get_env() -> Dict[str, str]:
        """
        Return the Gazebo plugin and resource path variables to set.

        Each variable extends its current value with the paths exported by packages.
        Plugins are also searched for in LD_LIBRARY_PATH, where ROS packages such as
        gz_ros2_control install them.
        """
        model_paths, plugin_paths = GazeboRosPaths.get_paths()
        return {
            'GZ_SIM_SYSTEM_PLUGIN_PATH': _join_paths(
                os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                os.environ.get('LD_LIBRARY_PATH', ''),
                plugin_paths),
            'GZ_SIM_RESOURCE_PATH': _join_paths(
                os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                model_paths),
        }


def _join_paths(*path_lists: str) -> str:
    """Join path lists, dropping empty and repeated entries (first occurrence wins)."""
    return os.pathsep.join(dict.fromkeys(
        path for path_list in path_lists for path in path_list.split(os.pathsep) if path))
