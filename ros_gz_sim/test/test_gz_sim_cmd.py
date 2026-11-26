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

import importlib.machinery
import importlib.util
import os

path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'gz_sim.launch.py.in')
loader = importlib.machinery.SourceFileLoader('gz_sim_launch', path)
spec = importlib.util.spec_from_loader('gz_sim_launch', loader)
gz_sim_launch = importlib.util.module_from_spec(spec)
loader.exec_module(gz_sim_launch)


def test_shell_true_keeps_list_unchanged():
    cmd = gz_sim_launch.build_gz_cmd('ruby gz sim', 'shapes.sdf -r', '8', 'true')
    assert cmd == ['ruby gz sim', 'shapes.sdf -r', '--force-version', '8']


def test_shell_false_splits_arguments():
    cmd = gz_sim_launch.build_gz_cmd('ruby /usr/bin/gz sim', 'shapes.sdf -r', '8', 'false')
    assert cmd == ['ruby', '/usr/bin/gz', 'sim', 'shapes.sdf', '-r', '--force-version', '8']
