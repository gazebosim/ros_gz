# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Check that every demo launch file loads and only references files that exist."""

import pathlib
import re

from launch import LaunchDescription
from launch.frontend import Parser
from launch.launch_description_sources import get_launch_description_from_python_launch_file
import pytest

PACKAGE_DIR = pathlib.Path(__file__).resolve().parent.parent
LAUNCH_FILES = sorted(
    p for p in (PACKAGE_DIR / 'launch').iterdir()
    if p.name.endswith(('.launch.py', '.launch.xml')))

# $(find-pkg-share ros_gz_sim_demos)/config/camera.yaml
XML_FILE_REF = re.compile(r'\$\(find-pkg-share ros_gz_sim_demos\)/([^"\s]+)')
# os.path.join(pkg_ros_gz_sim_demos, 'config', 'battery.yaml') or
# PathJoinSubstitution([pkg_ros_gz_sim_demos, 'worlds', 'vehicle.sdf'])
PY_FILE_REF = re.compile(
    r'(?:os\.path\.join\(|PathJoinSubstitution\(\[)\s*pkg_ros_gz_sim_demos\s*,([^)\]]*)')


def load(path):
    if path.name.endswith('.launch.py'):
        return get_launch_description_from_python_launch_file(str(path))
    root, parser = Parser.load(str(path))
    return parser.parse_description(root)


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda p: p.name)
def test_launch_file_loads(path):
    assert isinstance(load(path), LaunchDescription)


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda p: p.name)
def test_referenced_files_exist(path):
    # Skip commented out code, e.g. disabled rviz nodes.
    text = '\n'.join(
        line for line in path.read_text().splitlines()
        if not line.lstrip().startswith(('#', '<!--')))
    refs = XML_FILE_REF.findall(text)
    for args in PY_FILE_REF.findall(text):
        parts = re.findall(r"'([^']*)'", args)
        if parts:
            refs.append('/'.join(parts))
    missing = [ref for ref in refs if not (PACKAGE_DIR / ref).exists()]
    assert not missing, f'{path.name} references missing files: {missing}'
