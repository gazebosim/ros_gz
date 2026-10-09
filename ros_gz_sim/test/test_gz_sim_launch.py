# Copyright 2026 Ye-Seol Kwon
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

"""Check the shell boundary without requiring a running Gazebo server."""

from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext
import pytest


@pytest.fixture
def subject():
    path = Path(__file__).resolve().parents[1] / 'launch/gz_sim.launch.py.in'
    loader = SourceFileLoader('gz_launch_under_test', str(path))
    module = module_from_spec(spec_from_loader(loader.name, loader))
    loader.exec_module(module)
    return module


def command(subject, executable, legacy=False, platform_name='posix'):
    # Do not patch the process-wide os.name: pathlib and launch also use it.
    subject.os = SimpleNamespace(**{**os.__dict__, 'name': platform_name})
    context = LaunchContext()
    context.launch_configurations.update(
        gz_args='-s -r empty.sdf', gz_version='10', ign_args='',
        ign_version='6' if legacy else '', debugger='false',
        on_exit_shutdown='false', debug_env='false')
    with patch.object(subject.shutil, 'which', return_value=executable), \
            patch.object(subject.GazeboRosPaths, 'get_paths', return_value=('', '')):
        action = subject.launch_gz(context)[0]
    action.process_description.prepare(context, action)
    return ' '.join(action.process_description.final_cmd)


@pytest.mark.skipif(os.name != 'posix', reason='Requires a POSIX shell')
@pytest.mark.parametrize('legacy', [False, True])
@pytest.mark.parametrize('directory', [
    'plain', 'Application Support', "author's files", 'ocean;files',
    'literal$HOME', 'double"quote', '\u6d77\u6d0b \u7814\u7a76',
])
def test_executable_is_one_literal_shell_argument(subject, tmp_path, directory, legacy):
    # A Ruby stand-in records argv only; the production shell command is unchanged.
    ruby = tmp_path / 'ruby'
    ruby.write_text(
        '#!/bin/sh\nexec ' + shlex.quote(sys.executable) +
        " -c 'import json, sys; print(json.dumps(sys.argv[1:]))' \"$@\"\n")
    ruby.chmod(0o755)
    executable = str(tmp_path / directory / ('ign' if legacy else 'gz'))
    result = subprocess.run(
        command(subject, executable, legacy), shell=True, executable='/bin/sh',
        env={**os.environ, 'PATH': str(tmp_path) + os.pathsep + os.environ['PATH']},
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == [
        executable, 'gazebo' if legacy else 'sim', '-s', '-r', 'empty.sdf',
        '--force-version', '6' if legacy else '10']


@pytest.mark.parametrize('legacy', [False, True])
def test_plain_posix_command_is_unchanged(subject, legacy):
    name = 'ign' if legacy else 'gz'
    assert command(subject, '/usr/bin/' + name, legacy) == (
        'ruby /usr/bin/' + name + (' gazebo' if legacy else ' sim') +
        ' -s -r empty.sdf --force-version ' + ('6' if legacy else '10'))


@pytest.mark.parametrize('legacy', [False, True])
@pytest.mark.parametrize('suffix', ['.bat', '.BAT'])
def test_windows_command_keeps_existing_bat_handling(subject, legacy, suffix):
    # Command construction only: this does not run cmd.exe on a POSIX host.
    name = 'ign' if legacy else 'gz'
    path = 'C:\\Program Files\\Gazebo\\' + name
    assert command(subject, path + suffix, legacy, 'nt') == (
        'ruby ' + path + (' gazebo' if legacy else ' sim') +
        ' -s -r empty.sdf --force-version ' + ('6' if legacy else '10'))
