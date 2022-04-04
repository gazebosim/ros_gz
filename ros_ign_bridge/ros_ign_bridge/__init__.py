# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

import ros_ign_bridge.mappings

from rosidl_cmake import expand_template


def generate_cpp(output_path, template_dir):
    data = {}
    data['mappings'] = ros_ign_bridge.mappings.mappings()

    template_file = os.path.join(template_dir, 'get_mappings.cpp.em')
    output_file = os.path.join(output_path, 'get_mappings.cpp')
    data_for_template = {'mappings': data['mappings']}
    expand_template(template_file, data_for_template, output_file)

    unique_package_names = {
            mapping.ros2_package_name for mapping in data['mappings']}
    data['ros2_package_names'] = list(unique_package_names)

    template_file = os.path.join(template_dir, 'get_factory.cpp.em')
    output_file = os.path.join(output_path, 'get_factory.cpp')
    expand_template(template_file, data, output_file)

    for ros2_package_name in unique_package_names:
        data_pkg = {
                'ros2_package_name': ros2_package_name,
                'mappings': [
                    m for m in data['mappings']
                    if m.ros2_package_name == ros2_package_name
                ]
        }
        template_file = os.path.join(template_dir, 'pkg_factories.hpp.em')
        output_file = os.path.join(
            output_path, 'factories/%s.hpp' % ros2_package_name)
        expand_template(template_file, data_pkg, output_file)

        template_file = os.path.join(template_dir, 'pkg_factories.cpp.em')
        output_file = os.path.join(
            output_path, 'factories/%s.cpp' % ros2_package_name)
        expand_template(template_file, data_pkg, output_file)
