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

import os
import re
import shutil
import sys
import types

# --- BEGIN shared rosdoc2 setup ---------------------------------------------
# This section is identical in ros_gz_bridge/doc/conf.py and
# ros_gz_sim/doc/conf.py; change both together.  It can't live in a shared
# module because rosdoc2 documents each package on its own.


def _find_package_root():
    """
    Return the package root and whether rosdoc2 is running this file.

    rosdoc2 doesn't hand this file to Sphinx.  It writes its own conf.py, which
    runs ``exec(open("<this file>").read())``, so ``__file__`` names rosdoc2's
    conf.py and its exec line is the only way back to this file.  When
    sphinx-build is run on this directory directly, ``__file__`` is this file.
    """
    conf_py = os.path.abspath(__file__)
    with open(conf_py) as f:
        match = re.search(r'^exec\(open\("([^"]+)"\)', f.read(), re.MULTILINE)
    if match:
        conf_py = match.group(1)
    return os.path.dirname(os.path.dirname(conf_py)), match is not None


_package_root, _run_by_rosdoc2 = _find_package_root()

# Let autodoc import the package from source.  rosdoc2 only puts the package
# root on sys.path when the package has no conf.py of its own, so without this
# autodoc fails with "No module named ...", or documents a copy installed in an
# underlay instead of this source.
sys.path.insert(0, _package_root)

# rosdoc2 copies README.md into the Sphinx source tree, but not the images it
# links to.  Sphinx resolves those links relative to each page that shows the
# README: the root page, standard_docs/README and the README.md copy itself in
# standard_docs/original.  Copy the images next to each of them.
if _run_by_rosdoc2 and os.path.isdir(os.path.join(_package_root, 'images')):
    for _page_dir in ('', 'standard_docs', os.path.join('standard_docs', 'original')):
        _page_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), _page_dir)
        if os.path.isdir(_page_dir):
            shutil.copytree(
                os.path.join(_package_root, 'images'),
                os.path.join(_page_dir, 'images'),
                dirs_exist_ok=True)


def _expose_action(*args, **kwargs):
    """Stand-in for launch.frontend.expose_action, used as a class decorator."""
    return lambda cls: cls


# Tell autodoc this is imported, so it isn't documented as part of the package.
_expose_action.__module__ = 'launch.frontend'


class _StubModule(types.ModuleType):
    """Stand-in for a module that isn't available when building the docs."""

    def __getattr__(self, name):
        # Make up any name the package imports, so that importing something
        # new from a stubbed module doesn't break autodoc.  The class gets
        # __module__ set, so that autodoc can introspect it.
        if name.startswith('__'):
            raise AttributeError(name)
        stub = type(name, (object,), {'__module__': self.__name__})
        setattr(self, name, stub)
        return stub


def _install_stubs(names):
    """
    Stub modules that the package imports at import time.

    rosdoc2 overwrites ``autodoc_mock_imports`` with a list derived from the
    ``exec_depend``s, and Sphinx's mocking doesn't always cover
    ``from pkg.sub import Name``, so install stubs before autodoc runs.
    """
    for name in names:
        module = _StubModule(name)
        if name == 'launch.frontend':
            module.expose_action = _expose_action
        sys.modules.setdefault(name, module)

# --- END shared rosdoc2 setup -----------------------------------------------


_install_stubs([
    'launch',
    'launch.action',
    'launch.frontend',
    'launch.launch_context',
    'launch.some_substitutions_type',
    'launch.substitutions',
    'launch.utilities',
    'launch.utilities.type_utils',
    'launch_ros',
    'launch_ros.actions',
    'launch_ros.descriptions',
    'launch_ros.parameters_type',
    # Only used to generate code at build time.
    'rosidl_pycommon',
])

project = 'ros_gz_bridge'
copyright = '2022, Open Source Robotics Foundation, Inc.'  # noqa: A001
author = 'Open Source Robotics Foundation, Inc.'

# rosdoc2 adds sphinx.ext.intersphinx and its mapping.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'myst_parser',
]

autodoc_member_order = 'bysource'

source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

exclude_patterns = [
    '_build',
    'overview.rst',
    'api.rst',
    'user_api.rst',
    'conversions.rst',
    'launch_action.rst',
    'tutorials',
]

html_theme = 'sphinx_rtd_theme'

breathe_default_project = 'ros_gz_bridge Doxygen Project'
