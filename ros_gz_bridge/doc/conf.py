import os
import shutil
import sys
import types

# Stub buildtool-only Python modules that rosdoc2 does not mock automatically.
# ros_gz_bridge/__init__.py does `from rosidl_pycommon import expand_template`
# for build-time code generation; that package is not available at
# documentation build time.
_mod = types.ModuleType('rosidl_pycommon')
_mod.expand_template = lambda *args, **kwargs: None
sys.modules.setdefault('rosidl_pycommon', _mod)


# rosdoc2 overwrites ``autodoc_mock_imports`` with a list derived from
# ``exec_depends``, and Sphinx's submodule mocking does not always cover
# ``from pkg.sub import Name`` forms.  Install concrete stubs for the
# ``launch`` / ``launch_ros`` submodules that the launch action imports.
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
_install_stub(
    'launch.frontend',
    ['Entity', 'Parser', 'expose_action'],
)
_install_stub('launch.launch_context', ['LaunchContext'])
_install_stub('launch.some_substitutions_type', ['SomeSubstitutionsType'])
_install_stub('launch.substitutions', ['TextSubstitution'])
_install_stub('launch.utilities', ['ensure_argument_type'])
_install_stub(
    'launch.utilities.type_utils',
    ['normalize_typed_substitution', 'perform_typed_substitution'],
)
_install_stub('launch_ros')
_install_stub(
    'launch_ros.actions',
    ['ComposableNodeContainer', 'LoadComposableNodes', 'Node'],
)
_install_stub('launch_ros.descriptions', ['ComposableNode'])
_install_stub('launch_ros.parameters_type', ['SomeParameters'])
# expose_action is used as a decorator: @expose_action('name')
sys.modules['launch.frontend'].expose_action = lambda *a, **k: (lambda cls: cls)


# rosdoc2 copies this file into its own build tree and exec()s it from there,
# so __file__ points at that copy and offers no way back to the package source.
# What does: the generated conf.py puts the package root -- the parent of the
# `python_source` directory named in rosdoc2.yaml -- on sys.path before exec'ing
# this file, so pick it back out from there.
def _find_package_root():
    for _candidate in sys.path:
        if _candidate \
                and os.path.isfile(os.path.join(_candidate, 'package.xml')) \
                and os.path.isdir(os.path.join(_candidate, 'images')):
            return _candidate
    return None


_pkg_root = _find_package_root()

# rosdoc2 copies README.md into the Sphinx source root but not the images/
# directory next to it, so the README's `images/...` links resolve to nothing
# and Sphinx warns "image file not readable".  Copy them in beside the README.
if _pkg_root:
    _srcdir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in dir() \
        else os.getcwd()
    shutil.copytree(
        os.path.join(_pkg_root, 'images'),
        os.path.join(_srcdir, 'images'),
        dirs_exist_ok=True)

project = 'ros_gz_bridge'
copyright = '2022, Open Source Robotics Foundation, Inc.'
author = 'Open Source Robotics Foundation, Inc.'

# 'sphinx.ext.intersphinx' is required, not optional: rosdoc2 raises a
# RuntimeError if it is missing from `extensions`, and it supplies
# `intersphinx_mapping` itself, which is why none is set here.
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
