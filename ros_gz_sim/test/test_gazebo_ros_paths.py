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

"""Tests for GazeboRosPaths.get_env()."""

import os

import pytest
from ros_gz_sim.paths import GazeboRosPaths

PLUGIN_VAR = 'GZ_SIM_SYSTEM_PLUGIN_PATH'
RESOURCE_VAR = 'GZ_SIM_RESOURCE_PATH'


def join(*paths):
    return os.pathsep.join(paths)


@pytest.fixture
def exported_paths(monkeypatch):
    """Make get_paths() return fixed paths, as if exported by packages."""
    monkeypatch.setattr(
        GazeboRosPaths, 'get_paths',
        staticmethod(lambda: (join('/pkg/models', '/pkg/media'), '/pkg/plugins')))


def test_extends_current_values(monkeypatch, exported_paths):
    monkeypatch.setenv(PLUGIN_VAR, '/user/plugins')
    monkeypatch.setenv('LD_LIBRARY_PATH', join('/opt/lib', '/usr/lib'))
    monkeypatch.setenv(RESOURCE_VAR, '/user/models')

    assert GazeboRosPaths.get_env() == {
        PLUGIN_VAR: join('/user/plugins', '/opt/lib', '/usr/lib', '/pkg/plugins'),
        RESOURCE_VAR: join('/user/models', '/pkg/models', '/pkg/media'),
    }


def test_unset_variables_add_no_empty_entries(monkeypatch, exported_paths):
    for var in (PLUGIN_VAR, 'LD_LIBRARY_PATH', RESOURCE_VAR):
        monkeypatch.delenv(var, raising=False)

    assert GazeboRosPaths.get_env() == {
        PLUGIN_VAR: '/pkg/plugins',
        RESOURCE_VAR: join('/pkg/models', '/pkg/media'),
    }


def test_no_exported_paths(monkeypatch):
    monkeypatch.setattr(GazeboRosPaths, 'get_paths', staticmethod(lambda: ('', '')))
    monkeypatch.setenv(PLUGIN_VAR, '/user/plugins')
    monkeypatch.delenv('LD_LIBRARY_PATH', raising=False)
    monkeypatch.setenv(RESOURCE_VAR, '/user/models')

    assert GazeboRosPaths.get_env() == {
        PLUGIN_VAR: '/user/plugins',
        RESOURCE_VAR: '/user/models',
    }


def test_repeated_entries_are_dropped_in_order(monkeypatch, exported_paths):
    monkeypatch.setenv(PLUGIN_VAR, join('/pkg/plugins', '/user/plugins'))
    monkeypatch.setenv('LD_LIBRARY_PATH', join('/user/plugins', '/opt/lib'))
    monkeypatch.setenv(RESOURCE_VAR, '/pkg/media')

    assert GazeboRosPaths.get_env() == {
        PLUGIN_VAR: join('/pkg/plugins', '/user/plugins', '/opt/lib'),
        RESOURCE_VAR: join('/pkg/media', '/pkg/models'),
    }


def test_applying_twice_does_not_grow(monkeypatch, exported_paths):
    # GzServer writes the result back into os.environ, so every later GzServer in
    # the same launch starts from the previous result.
    monkeypatch.setenv(PLUGIN_VAR, '/user/plugins')
    monkeypatch.setenv('LD_LIBRARY_PATH', '/opt/lib')
    monkeypatch.setenv(RESOURCE_VAR, '/user/models')

    first = GazeboRosPaths.get_env()
    for var, value in first.items():
        monkeypatch.setenv(var, value)

    assert GazeboRosPaths.get_env() == first
