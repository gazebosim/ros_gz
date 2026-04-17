import inspect
import os
import re
import sys
import types

# Stub buildtool-only Python modules that rosdoc2 does not mock automatically.
# ros_gz_bridge/__init__.py imports rosidl_pycommon for build-time code
# generation; it is not available at documentation build time.
for _name in ('rosidl_pycommon', 'em'):
    _mod = types.ModuleType(_name)
    # __init__.py does: from rosidl_pycommon import expand_template
    _mod.expand_template = lambda *args, **kwargs: None
    sys.modules.setdefault(_name, _mod)


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

# When rosdoc2 runs sphinx-build, it exec's this file from a generated wrapper.
# Parse the user_conf_py path out of the call stack and add the package root
# to sys.path so autodoc can import ros_gz_bridge.
try:
    import ros_gz_bridge  # noqa: F401
except ImportError:
    for _fi in inspect.stack():
        for _line in (_fi.code_context or []):
            _m = re.search(r'exec\(open\("([^"]+)"\)', _line)
            if _m:
                _pkg_root = os.path.dirname(os.path.dirname(_m.group(1)))
                if os.path.isdir(_pkg_root):
                    sys.path.insert(0, _pkg_root)
                break

project = 'ros_gz_bridge'
copyright = '2022, Open Source Robotics Foundation, Inc.'
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
    'conversions.rst',
    'launch_action.rst',
    'tutorials',
]

html_theme = 'sphinx_rtd_theme'

breathe_default_project = 'ros_gz_bridge Doxygen Project'
