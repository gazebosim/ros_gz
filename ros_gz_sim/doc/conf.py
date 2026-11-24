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

import inspect
import os
import re
import sys
import types

# When rosdoc2 runs sphinx-build, it exec's this file from a generated wrapper.
# Parse the user_conf_py path out of the call stack and add the package root
# to sys.path so autodoc can import ros_gz_sim.
try:
    import ros_gz_sim  # noqa: F401
except ImportError:
    for _fi in inspect.stack():
        for _line in (_fi.code_context or []):
            _m = re.search(r'exec\(open\("([^"]+)"\)', _line)
            if _m:
                _pkg_root = os.path.dirname(os.path.dirname(_m.group(1)))
                if os.path.isdir(_pkg_root):
                    sys.path.insert(0, _pkg_root)
                break


# rosdoc2 overwrites ``autodoc_mock_imports`` with a list derived from
# ``exec_depends``, and Sphinx's submodule mocking does not always cover
# ``from pkg.sub import Name`` forms.  Install concrete stubs for the
# ``launch`` / ``launch_ros`` submodules that the launch actions import.
def _install_stub(name, attrs=()):
    module = types.ModuleType(name)
    for attr in attrs:
        # Build a real class with __module__ set so autodoc signature
        # introspection does not choke on the stub.
        cls = type(attr, (object,), {'__module__': name})
        setattr(module, attr, cls)
    sys.modules.setdefault(name, module)


_install_stub('launch')
_install_stub('launch.action', ['Action'])
_install_stub('launch.actions', ['DeclareLaunchArgument', 'ExecuteProcess',
                                 'IncludeLaunchDescription', 'OpaqueFunction',
                                 'RegisterEventHandler',
                                 'SetEnvironmentVariable'])
_install_stub('launch.conditions', ['IfCondition', 'UnlessCondition'])
_install_stub(
    'launch.frontend',
    ['Entity', 'Parser', 'expose_action'],
)
_install_stub('launch.launch_context', ['LaunchContext'])
_install_stub('launch.launch_description', ['LaunchDescription'])
_install_stub('launch.launch_description_sources',
              ['PythonLaunchDescriptionSource'])
_install_stub('launch.some_substitutions_type', ['SomeSubstitutionsType'])
_install_stub('launch.substitutions',
              ['LaunchConfiguration', 'PathJoinSubstitution',
               'TextSubstitution'])
_install_stub('launch.utilities', ['ensure_argument_type',
                                   'normalize_to_list_of_substitutions',
                                   'perform_substitutions'])
_install_stub('launch.utilities.type_utils',
              ['normalize_typed_substitution', 'perform_typed_substitution'])
_install_stub('launch_ros')
_install_stub('launch_ros.actions',
              ['ComposableNodeContainer', 'LoadComposableNodes', 'Node',
               'SetParameter'])
_install_stub('launch_ros.descriptions', ['ComposableNode'])
_install_stub('launch_ros.parameters_type', ['SomeParameters'])
_install_stub('launch_ros.substitutions', ['FindPackageShare'])
sys.modules['launch.frontend'].expose_action = lambda *a, **k: (lambda cls: cls)

# Stub ament_index_python because it may not be importable in the doc env.
_install_stub('ament_index_python', ['get_package_share_directory'])
_install_stub('ament_index_python.packages',
              ['get_package_share_directory', 'get_packages_with_prefixes'])

project = 'ros_gz_sim'
copyright = '2022, Open Source Robotics Foundation, Inc.'  # noqa: A001
author = 'Open Source Robotics Foundation, Inc.'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode',
    'myst_parser',
]

autodoc_member_order = 'bysource'

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

templates_path = ['_templates']

exclude_patterns = [
    '_build',
    'overview.rst',
    'api.rst',
    'user_api.rst',
    'launch_actions.rst',
    'executables.rst',
    'tutorials',
]

html_theme = 'sphinx_rtd_theme'

breathe_default_project = 'ros_gz_sim Doxygen Project'
