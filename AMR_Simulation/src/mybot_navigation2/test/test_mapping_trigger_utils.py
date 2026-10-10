import importlib.util
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'src' / 'mapping_trigger_utils.py'

spec = importlib.util.spec_from_file_location('mapping_trigger_utils', MODULE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_parse_command_from_json_string():
    payload = '{"data": {"command": "start"}}'
    assert module.parse_command(payload) == 'start'


def test_parse_command_from_plain_string():
    assert module.parse_command('stop') == 'stop'


def test_parse_command_from_nested_dict():
    payload = {'data': {'command': 'start'}}
    assert module.parse_command(payload) == 'start'
