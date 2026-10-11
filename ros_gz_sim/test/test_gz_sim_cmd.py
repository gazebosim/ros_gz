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
