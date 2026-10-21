"""ROS-free checks of full logging, supervisor order, failures and archive content."""
import argparse
import csv
import hashlib
import importlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import sys
import tarfile
import tempfile
import types
from unittest.mock import patch
import yaml


def validate(bundle):
    package = bundle / 'roundabout_mission'
    sys.path.insert(0, str(package / 'src'))
    from roundabout_mission import VERSION
    from roundabout_mission.ros_io import CsvLog
    from roundabout_mission.trial_tools import apply_profile, summarize, sha256

    checks = []
    sample = dict(timestamp=1., version=VERSION, state='COMMIT', allowed=True,
                  stop_required=False, emergency=False, decision_reason='commit_latched',
                  decision_reason_detail='commit', lead_vehicle_id=10, lead_gap=1.3,
                  desired_gap=6.1, minimum_ttc=.79, actual_target_speed=2.,
                  accel_cmd=0., brake_cmd=.3, steering_cmd=0.,
                  lead_prediction_suppressed_ids=[10],
                  lead_prediction_stop_suppressed=True,
                  lead_relative_following_active=True,
                  lead_gap_control_mode='relative_speed',
                  lead_relative_speed_cap_mps=2.,
                  custom_nested={'인지': [{'id': 10, 'x': 19.3, 'v': 2.}]})
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        log = CsvLog(str(root), 'control')
        original = dict(sample)
        for i in range(3):
            log.write(dict(sample, timestamp=1.+i*.1))
        log.close()
        csv_file = next(root.glob('control_*.csv'))
        json_file = next(root.glob('control_*.jsonl'))
        assert csv_file.stem == json_file.stem
        rows = [json.loads(line) for line in json_file.read_text().splitlines()]
        assert len(rows) == 3 and sample == original
        for key, value in sample.items():
            assert rows[0][key] == value, key
        assert all(isinstance(rows[0][key], float) for key in ('log_wall_time_s', 'log_monotonic_s'))
        with csv_file.open(newline='') as stream:
            saved = list(csv.DictReader(stream))
        assert len(saved) == 3 and saved[0]['brake_cmd'] == '0.3'
        if VERSION == '6.1.0':
            assert json.loads(saved[0]['lead_prediction_suppressed_ids']) == [10]
            assert saved[0]['lead_prediction_stop_suppressed'] == 'True'
        assert summarize(root)['control_samples'] == 3
    checks.append('paired CSV/JSONL preserve commands, nested fields, clocks and V6.1 diagnostics')

    disabled = CsvLog('', 'control')
    disabled.write(sample)
    disabled.close()
    assert disabled.stream is None and disabled.json_stream is None
    checks.append('disabled file logging remains supported')
    config_name = 'v61_speedtest.yaml' if VERSION == '6.1.0' else 'v6_speedtest.yaml'
    supplied = yaml.safe_load((bundle / config_name).read_text())
    assert apply_profile(supplied, 'baseline') == supplied
    for profile in ('candidate', 'scheduled'):
        try:
            apply_profile(supplied, profile)
        except ValueError:
            pass
        else:
            raise AssertionError('unsupported driving profile was accepted')
    checks.append('baseline recorder preserves supplied driving parameters and rejects other profiles')
    protected = json.loads((bundle / 'validation/driving_hashes_before_logging.json').read_text())
    for relative, digest in protected.items():
        assert sha256(bundle / relative) == digest, relative
    checks.append('core, gate, actual driver and driving YAML hashes are unchanged by logging')

    spec = importlib.util.spec_from_file_location('checked_recorder', package / 'scripts/roundabout_record_trial.py')
    recorder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recorder)

    def simulate(master_initially_up, bag_fails=False):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            routes = [root / 'path.csv', root / 'circulating.csv']
            for route in routes:
                route.write_text('0,0,0\n1,0,0\n')
            config = dict(supplied, path_file=str(routes[0]), circulating_path_file=str(routes[1]))
            cfg_file = root / 'config.yaml'
            cfg_file.write_text(yaml.safe_dump(config))
            deps = {}
            for name in ('purepursuit_mgeo', 'path_planning'):
                dep = root / name
                (dep / 'src').mkdir(parents=True)
                (dep / 'src/reference.py').write_text('recorded_dependency = True\n')
                (dep / 'package.xml').write_text('<package/>\n')
                deps[name] = dep
            env = dict(up=master_initially_up, ready=False, launch=False, topics=[], events=[], processes={}, logs=[])

            class Master:
                def getPid(self):
                    if not env['up']:
                        raise RuntimeError('not running')
                    return 1
                def getSystemState(self):
                    subscribers = [(t, ['/roundabout_trial_bag']) for t in env['topics']] if env['ready'] else []
                    if subscribers:
                        env['events'].append('subscriptions_ready')
                    return [[], subscribers, []]
                def getParam(self, name):
                    return config if name != '/use_sim_time' else False
                def hasParam(self, name):
                    return name == '/use_sim_time'
                def getTopicTypes(self):
                    return [(t, 'mock/RecordedTopic') for t in env['topics']]

            class Process:
                def __init__(self, command, **kwargs):
                    self.name = command[0]
                    self.pid = 100+len(env['processes'])
                    self.returncode = None
                    env['processes'][self.pid] = self
                    env['events'].append('start_'+self.name)
                    if self.name == 'roscore':
                        env['up'] = True
                    elif self.name == 'rosbag':
                        at = command.index('-O')
                        self.active = Path(command[at+1]).with_name('traffic_0.bag.active')
                        self.active.write_bytes(b'MOCK_BAG_FOR_SUPERVISOR_TEST')
                        env['trial'] = self.active.parent
                        env['topics'] = command[at+2:-1]
                        env['ready'] = True
                    elif self.name == 'roslaunch':
                        assert 'subscriptions_ready' in env['events'], 'driving started before recording'
                        env['launch'] = True
                        for prefix in ('decision', 'control'):
                            log = CsvLog(str(env['trial']), prefix)
                            for i in range(3):
                                log.write(dict(sample, timestamp=1.+i*.1))
                            env['logs'].append(log)
                def poll(self):
                    if self.name == 'rosbag' and bag_fails and env['launch']:
                        self.returncode = 2
                    return self.returncode
                def wait(self, timeout=None):
                    if self.name == 'roslaunch':
                        for log in env['logs']:
                            log.write(dict(sample, timestamp=1.3, brake_cmd=1., actual_target_speed=0.,
                                           stop_required=True, decision_reason='shutdown'))
                            log.close()
                    elif self.name == 'rosbag':
                        self.active.rename(self.active.with_suffix(''))
                    self.returncode = 0
                    return self.returncode

            def killpg(pid, sig):
                assert sig == signal.SIGINT
                env['events'].append('stop_'+env['processes'][pid].name)

            def sleep(seconds):
                if seconds == .2:
                    raise KeyboardInterrupt
                raise AssertionError('startup polling unexpectedly needed')

            class Bag:
                def __init__(self, path):
                    assert Path(path).is_file()
                def __enter__(self):
                    return self
                def __exit__(self, *args):
                    pass
                def get_type_and_topic_info(self):
                    return types.SimpleNamespace(topics={t: types.SimpleNamespace(message_count=4) for t in env['topics']})

            rosgraph = types.SimpleNamespace(Master=lambda name: Master())
            rospkg = types.SimpleNamespace(RosPack=lambda: types.SimpleNamespace(
                get_path=lambda name: str(package if name == 'roundabout_mission' else deps[name])))
            output = io.StringIO()
            argv = ['recorder', '--config', str(cfg_file), '--morai-host-ip', '192.168.0.147',
                    '--output-root', str(root / 'trials'), '--note', 'mock test', '--launch-arg', 'rviz:=false']
            with patch.dict(sys.modules, rosgraph=rosgraph, rospkg=rospkg, rosbag=types.SimpleNamespace(Bag=Bag)), \
                 patch.object(sys, 'argv', argv), patch.object(recorder.shutil, 'which', return_value='/mock/executable'), \
                 patch.object(recorder.subprocess, 'Popen', side_effect=Process), \
                 patch.object(recorder.os, 'killpg', side_effect=killpg), \
                 patch.object(recorder.time, 'sleep', side_effect=sleep), patch.object(sys, 'stdout', output), \
                 patch.object(sys, 'stderr', output):
                result = recorder.main()

            trial = env['trial']
            metadata = json.loads((trial / 'metadata.json').read_text())
            assert metadata['code_version'] == VERSION
            assert trial.name.startswith('v'+VERSION.replace('.', '_')+'_')
            assert metadata['control_samples'] == 4
            assert sha256(trial / 'sources/roundabout_mission/src/roundabout_mission/core.py') == protected['roundabout_mission/src/roundabout_mission/core.py']
            runtime = yaml.safe_load((trial / 'effective_config.yaml').read_text())
            expected = dict(config, evaluation_mode=False, enable_control=True, log_dir=str(trial))
            assert runtime == expected
            assert env['events'].index('subscriptions_ready') < env['events'].index('start_roslaunch')
            assert (trial / 'sources/path_planning/src/reference.py').is_file()
            required = ['metadata.json', 'summary.json', 'effective_config.yaml', 'supplied_config.yaml',
                        'path_file.csv', 'circulating_path_file.csv', 'launch_console.log', 'rosbag_console.log']
            assert all((trial / f).is_file() for f in required)
            archive_path = Path(next(s.split('=', 1)[1] for s in output.getvalue().splitlines() if s.startswith('SEND_THIS_FILE=')))
            with tarfile.open(archive_path) as archive:
                assert all(trial.name+'/'+f in archive.getnames() for f in required)
                assert json.load(archive.extractfile(trial.name+'/metadata.json')) == metadata
                assert sum(f.name.endswith('.jsonl') for f in archive.getmembers()) == 2
            if bag_fails:
                assert result == 1 and 'Recorder stopped unexpectedly' in metadata['error']
                assert metadata['unfinished_bags'] and metadata['missing_or_empty_topics']
                assert 'INCOMPLETE' in output.getvalue()
                assert 'stop_roslaunch' in env['events']
            else:
                assert result == 0 and metadata['interrupted_by_user']
                assert not metadata['unfinished_bags'] and not metadata['missing_or_empty_topics']
                assert 'INCOMPLETE' not in output.getvalue()
                assert (trial / 'runtime_params.yaml').is_file()
                assert env['events'].index('stop_roslaunch') < env['events'].index('stop_rosbag')
                if not master_initially_up:
                    assert env['events'].index('stop_rosbag') < env['events'].index('stop_roscore')
                final = [json.loads(s) for s in next(trial.glob('control_*.jsonl')).read_text().splitlines()][-1]
                assert final['brake_cmd'] == 1. and final['decision_reason'] == 'shutdown'
            return env['events']

    simulate(True)
    checks.append('running-master trial starts recording first, saves final brake, snapshots and complete archive')
    simulate(False)
    checks.append('absent-master trial starts roscore and shuts it down after the final bag')
    simulate(True, bag_fails=True)
    checks.append('recorder failure stops driving, marks incomplete recording and still creates diagnostic archive')
    report = dict(version=VERSION, status='pass', checks_count=len(checks), checks=checks,
                  runtime_limitations='ROS transport/processes/bag reading were mocked; ROS Noetic/MORAI were not run.')
    (bundle / 'validation/logging_checks.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle', type=Path, default=Path(__file__).resolve().parents[1])
    validate(parser.parse_args().bundle.resolve())
