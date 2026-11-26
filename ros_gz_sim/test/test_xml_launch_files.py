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

"""Check that every XML launch file parses and is installed."""

import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.frontend import Parser
import pytest

LAUNCH_DIR = pathlib.Path(__file__).resolve().parent.parent / 'launch'
XML_LAUNCH_FILES = sorted(LAUNCH_DIR.glob('*.launch'))


@pytest.mark.parametrize('path', XML_LAUNCH_FILES, ids=lambda p: p.name)
def test_xml_launch_file_parses(path):
    # Parse with the XML frontend directly, so that a failure reports the XML
    # error instead of the last parser tried by get_launch_description_from_any_launch_file.
    root, parser = Parser.load(str(path))
    assert isinstance(parser.parse_description(root), LaunchDescription)


@pytest.mark.parametrize('path', XML_LAUNCH_FILES, ids=lambda p: p.name)
def test_xml_launch_file_installed(path):
    share = pathlib.Path(get_package_share_directory('ros_gz_sim'))
    assert (share / 'launch' / path.name).is_file()
