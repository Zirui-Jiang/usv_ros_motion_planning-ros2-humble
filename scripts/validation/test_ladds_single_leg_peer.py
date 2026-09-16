"""The validation sender must not turn feedback waiting into repeated voyages."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


def test_peer_sends_once_while_waiting_several_seconds_for_completion(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        'single_leg_peer_under_test', Path(__file__).with_name('ladds_single_leg_peer.py'))
    peer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(peer)
    clock = [0.0]
    sent = []

    class Client:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            pass

        def close(self):
            pass

        def send(self, payload):
            sent.append(json.loads(payload))

        def receive(self):
            if clock[0] >= 2.5:
                return SimpleNamespace(
                    topic='0052', payload=b'{"travelId":58,"taskFinish":1}')
            return None

    monkeypatch.setattr(peer, 'LaDDSClient', Client)
    monkeypatch.setattr(peer.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(peer.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    task_file = tmp_path / 'task.json'
    output = tmp_path / 'result.json'
    task_file.write_text('{"travelId":58}', encoding='utf-8')
    monkeypatch.setattr(peer.sys, 'argv', [
        'peer', '--library', str(tmp_path / 'LaDDS.dll'),
        '--task-file', str(task_file), '--output', str(output),
        '--warmup', '0', '--duration', '5'])

    assert peer.main() == 0

    assert sent == [{'travelId': 58}]
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['sent_count'] == 1
    assert result['success'] is True
