#!/usr/bin/env python3
"""Start recording before driving, then save one complete trial on Ctrl+C.

Uses the existing standalone launch and preserves driving parameters.
Only the baseline recording profile is supported in V6/V6.1.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import yaml
from roundabout_mission import VERSION
from roundabout_mission.trial_tools import apply_profile, sha256, summarize, package_trial


def stop_process(process, timeout=20):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--morai-host-ip', required=True)
    parser.add_argument('--profile', choices=['baseline'], default='baseline')
    parser.add_argument('--output-root', default='/root/roundabout_trials')
    parser.add_argument('--note', default='')
    parser.add_argument('--extra-topic', action='append',default=[],
                        help='Additional known simulator status topic to record; repeatable')
    parser.add_argument('--launch-arg', action='append', default=[], help='Extra standalone arg, e.g. lidar_port:=2001')
    args=parser.parse_args()
    source=Path(args.config).expanduser().resolve()
    config=yaml.safe_load(source.read_text(encoding='utf-8'))
    config=apply_profile(config,args.profile)
    config['evaluation_mode']=False
    config['enable_control']=True
    # Validate route presence before starting any controller.
    for key in ('path_file','circulating_path_file'):
        route=Path(config[key]).expanduser().resolve()
        if not route.is_file():
            parser.error('Missing '+key+': '+str(route))
        config[key]=str(route)
    for arg in args.launch_arg:
        if ':=' not in arg or arg.split(':=',1)[0] in (
                'mission_config','enable_control','evaluation_mode','run_driver','log_dir',
                'morai_host_ip','command_topic'):
            parser.error('Unsupported/managed launch argument: '+arg)
    for command in ('roslaunch','rosbag','roscore'):
        if not shutil.which(command):
            parser.error(command+' not found. Source ROS and workspace setup.bash first.')
    import rosgraph
    import rospkg
    package=Path(rospkg.RosPack().get_path('roundabout_mission'))
    master=rosgraph.Master('/roundabout_trial_supervisor')
    def system_state():
        return master.getSystemState()
    def master_up():
        try:
            master.getPid()
            return True
        except Exception:
            return False
    if master_up():
        nodes={node for group in system_state() for _,owners in group for node in owners}
        if nodes.intersection({'/roundabout_merge_gate','/roundabout_test_driver','/roundabout_trial_bag'}):
            parser.error('Existing roundabout launch/recorder detected; stop it before this trial.')
        command_topic=config.get('command_topic','/ctrl_cmd')
        for topic,owners in system_state()[0]:
            if topic==command_topic and owners:
                parser.error('Another controller publishes '+command_topic+': '+str(owners))
    label='v'+VERSION.replace('.','_')+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+args.profile+'_'+str(os.getpid())
    trial=Path(args.output_root).expanduser().resolve()/label
    trial.mkdir(parents=True,exist_ok=False)
    config['log_dir']=str(trial)
    config_path=trial/'effective_config.yaml'
    config_path.write_text(yaml.safe_dump(config,sort_keys=False),encoding='utf-8')
    shutil.copy2(str(source),str(trial/'supplied_config.yaml'))
    sources=trial/'sources'
    shutil.copytree(str(package),str(sources/'roundabout_mission'),
                    ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git','docs','test'))
    # Capture actual externally imported driving dependencies as well.
    for name in ('purepursuit_mgeo','path_planning'):
        dep=Path(rospkg.RosPack().get_path(name))
        for sub in ('src','package.xml','CMakeLists.txt'):
            src=dep/sub; dst=sources/name/sub
            if src.is_dir():
                shutil.copytree(str(src),str(dst),ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
            elif src.is_file():
                dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(str(src),str(dst))
    for key in ('path_file','circulating_path_file'):
        shutil.copy2(config[key],str(trial/(key+Path(config[key]).suffix)))
    topics=list(dict.fromkeys([
        config.get('odom_topic','/localization/odometry'),
        config.get('obstacle_topic','/perception/lidar/tracked_obstacles_map'),
        config.get('command_topic','/ctrl_cmd'),
        '/roundabout_merge_gate/status','/roundabout_test_driver/status',
        '/roundabout_merge_gate/stop_required','/roundabout_merge_gate/allowed',
        '/roundabout_merge_gate/target_speed','/roundabout_test_driver/command_preview',
        '/roundabout_trial/event','/clock','/rosout_agg']+args.extra_topic))
    metadata=dict(code_version=VERSION,profile=args.profile,note=args.note,start_utc=datetime.now(timezone.utc).isoformat(),
        start_wall_s=time.time(),topics=topics,package_path=str(package),
        ros_master_uri=os.environ.get('ROS_MASTER_URI'),
        parameters=config,launch_args=args.launch_arg,
        profile_description='Full trial recording; supplied driving parameters are preserved.')
    metadata['input_hashes']={str(f.relative_to(trial)):sha256(f) for f in trial.rglob('*') if f.is_file()}
    meta=trial/'metadata.json'
    def save_metadata():
        meta.write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    save_metadata()
    core=bag=launch=None
    streams=[]
    interrupted=False
    exit_code=0
    def spawn(command,filename):
        stream=(trial/filename).open('w',encoding='utf-8');streams.append(stream)
        return subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    def wait_until(test,process,seconds=15):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError('Process exited during startup. Read logs in '+str(trial))
            if test():
                return
            time.sleep(.1)
        raise RuntimeError('Startup timeout. Read logs in '+str(trial))
    def term_handler(signum,frame):
        raise KeyboardInterrupt
    old_int=signal.getsignal(signal.SIGINT)
    old_term=signal.signal(signal.SIGTERM,term_handler)
    try:
        print('TRIAL_DIRECTORY='+str(trial),flush=True)
        if not master_up():
            core=spawn(['roscore'],'roscore_console.log')
            wait_until(master_up,core)
        bag=spawn(['rosbag','record','--buffsize=256','--split','--size=256',
                   '-O',str(trial/'traffic.bag')]+topics+['__name:=roundabout_trial_bag'], 'rosbag_console.log')
        def recording_ready():
            subscribers={topic:owners for topic,owners in system_state()[1]}
            return all('/roundabout_trial_bag' in subscribers.get(t,[]) for t in topics)
        wait_until(recording_ready,bag)
        command=['roslaunch','roundabout_mission','roundabout_standalone.launch',
            'mission_config:='+str(config_path),'enable_control:=true',
            'evaluation_mode:=false','morai_host_ip:='+args.morai_host_ip,
            'command_topic:='+config.get('command_topic','/ctrl_cmd'),'log_dir:='+str(trial)]+args.launch_arg
        metadata['launch_command']=command;save_metadata()
        launch=spawn(command,'launch_console.log')
        print('Recording started BEFORE driving. Profile='+args.profile,flush=True)
        print('Finish the run, then press Ctrl+C ONCE here. Keep MORAI running until the vehicle stops.',flush=True)
        params_saved=False
        while launch.poll() is None:
            if bag.poll() is not None:
                raise RuntimeError('Recorder stopped unexpectedly; stopping this test.')
            if not params_saved:
                try:
                    snapshot={name:master.getParam(name) for name in ('/roundabout_merge_gate','/roundabout_test_driver')}
                    (trial/'runtime_params.yaml').write_text(yaml.safe_dump(snapshot,sort_keys=False),encoding='utf-8')
                    metadata['runtime_topic_types']=master.getTopicTypes()
                    metadata['use_sim_time']=master.getParam('/use_sim_time') if master.hasParam('/use_sim_time') else False
                    save_metadata();params_saved=True
                except Exception:
                    pass
            time.sleep(.2)
        metadata['launch_return_code']=launch.returncode
        if launch.returncode:
            raise RuntimeError('roslaunch failed. See launch_console.log')
    except KeyboardInterrupt:
        interrupted=True
    except Exception as exc:
        metadata['error']=str(exc);exit_code=1
        print('ERROR: '+str(exc),file=sys.stderr,flush=True)
    finally:
        # Finish the controller first, so its final brake is also recorded.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        stop_process(launch)
        stop_process(bag,60)
        stop_process(core)
        for stream in streams: stream.close()
        metadata.update(end_utc=datetime.now(timezone.utc).isoformat(),
                        interrupted_by_user=interrupted,bag_return_code=None if bag is None else bag.returncode)
        metadata['unfinished_bags']=[f.name for f in trial.glob('*.active')]
        try:
            import rosbag
            counts={}
            for path in trial.glob('*.bag'):
                with rosbag.Bag(str(path)) as recorded:
                    for topic,info in recorded.get_type_and_topic_info().topics.items():
                        counts[topic]=counts.get(topic,0)+info.message_count
            metadata['bag_message_counts']=counts
            metadata['missing_or_empty_topics']=[t for t in topics[:5] if not counts.get(t)]
        except Exception as exc:
            metadata['bag_inspection_error']=str(exc)
        save_metadata()
        try:
            summary=summarize(trial)
            metadata['control_samples']=summary['control_samples'];save_metadata()
            print('Control samples saved: '+str(summary['control_samples']),flush=True)
        except Exception as exc:
            metadata['summary_error']=str(exc);save_metadata()
        if (metadata.get('missing_or_empty_topics') or metadata['unfinished_bags']
                or metadata.get('bag_inspection_error') or not metadata.get('control_samples')):
            print('Recording is INCOMPLETE; send the bundle anyway for diagnosis.',flush=True)
        print('Packing this trial...',flush=True)
        output=package_trial(trial)
        print('SEND_THIS_FILE='+str(output),flush=True)
        signal.signal(signal.SIGINT,old_int)
        signal.signal(signal.SIGTERM,old_term)
    return exit_code


if __name__=='__main__':
    sys.exit(main())
