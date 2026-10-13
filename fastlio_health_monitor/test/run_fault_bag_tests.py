#!/usr/bin/env python3
"""ROS integration tests using generated fault copies and the real C++ V2 node."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rospy,yaml
from diagnostic_msgs.msg import DiagnosticArray

def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

def main():
    repo_root=Path(__file__).resolve().parents[2]
    ap=argparse.ArgumentParser();ap.add_argument('--cases',default=str(Path.home()/'datasets/fastlio/health_fault_cases'));ap.add_argument('--evidence',default=str(repo_root/'FAST_LIO/doc/ros1_workspace/evidence/v2'));a=ap.parse_args();out=Path(a.evidence);out.mkdir(parents=True,exist_ok=True)
    config_path=Path(__file__).resolve().parents[1]/'config/health_v2_avia.yaml';manifest=json.loads((Path(a.cases)/'manifest.json').read_text())
    rospy.init_node('v2_fault_bag_test',anonymous=True,disable_signals=True)
    rospy.set_param('/use_sim_time',True)
    snapshots=[]
    def cb(m):
        snapshots.append({'received_monotonic':time.monotonic(),'ros_stamp_sec':m.header.stamp.to_sec(),'statuses':[{'name':s.name,'level':int(s.level),'message':s.message,'values':{v.key:v.value for v in s.values}} for s in m.status]})
    sub=rospy.Subscriber('/health_test/diagnostics',DiagnosticArray,cb,queue_size=100)
    results=[]
    for case in manifest['cases']:
        name=case['name'];snapshots.clear();c=yaml.safe_load(config_path.read_text());c['monitor']['data_reference']=case['bag'];c['monitor']['diagnostics_topic']='/health_test/diagnostics';c['monitor']['status_topic']='/health_test/status';c['monitor']['soft_warning_hold_sec']=.1;c['monitor']['recovery_hold_sec']=.3;c['monitor']['min_frequency_coverage_sec']=.5;c['logging']['directory']=str(out/'runtime_logs');c['modules'][2]['topic']='/health_test/odom';c['lifecycle_defaults']['ready_consecutive_samples']=5
        path=out/(name+'.yaml');path.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False))
        monitor=player=None
        try:
            with (out/(name+'_node.log')).open('w') as log:
                monitor=subprocess.Popen(['rosrun','fastlio_health_monitor','robot_health_monitor_node','__name:=v2_fault_monitor','_config_file:='+str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+5
                while not snapshots and time.monotonic()<deadline and monitor.poll() is None:time.sleep(.05)
                assert monitor.poll() is None,'monitor failed startup'
                with (out/(name+'_player.log')).open('w') as plog:
                    player=subprocess.Popen(['rosbag','play','--clock','--delay=.3','--rate=2',case['bag']],stdout=plog,stderr=subprocess.STDOUT,start_new_session=True)
                    player.wait(timeout=25)
                time.sleep(1.1)
                stop(monitor)
            assert snapshots,'no diagnostics'
            logdir=next(s['values']['log_directory'] for snap in snapshots for s in snap['statuses'] if s['name']=='Robot health/overall')
            events=[json.loads(line) for p in sorted(Path(logdir).glob('events.*.jsonl')) for line in p.read_text().splitlines()]
            triggered=[e for e in events if e['event_type']=='RULE_TRIGGERED']
            rules={e['rule_id'] for e in triggered};expected=case['expected_rule'];checks={}
            if name=='baseline_short':
                checks['imu_running_ok']=any(s['name']=='Robot health/imu' and s['level']==0 and s['values'].get('operating_state')=='RUNNING' for snap in snapshots for s in snap['statuses'])
                checks['localization_running_ok']=any(s['name']=='Robot health/localization' and s['level']==0 and s['values'].get('operating_state')=='RUNNING' for snap in snapshots for s in snap['statuses'])
            elif name=='quaternion_sign_valid':checks['no_sign_jump']=not ({'LOC_ROTATION_JUMP','LOC_ANGULAR_SPEED'}&rules)
            else:
                module='lidar' if name.startswith('lidar') else 'localization' if name.startswith('localization') else 'imu'
                checks['expected_event']=any(e['event_type']=='RULE_TRIGGERED' and e['module_id']==module and e['rule_id']==expected for e in events)
                module='lidar' if name.startswith('lidar') else 'localization' if name.startswith('localization') else 'imu'
                checks['diagnostic_visible']=any(s['name']=='Robot health/'+module and expected in s['message'] and s['level']>0 for snap in snapshots if manifest['bag_start_time_sec']+case['fault_interval_sec'][0]-.3<=snap['ros_stamp_sec']<manifest['bag_start_time_sec']+min(case['fault_interval_sec'][1]+2,manifest['duration_sec']-.3) for s in snap['statuses'])
                if name not in ('lidar_dropout','lidar_low_frequency'):checks['rule_recovers']=any(e['event_type']=='RULE_RECOVERED' and e['rule_id']==expected for e in events)
            checks['events_in_order']=all(a['event_seq']<b['event_seq'] for a,b in zip(events,events[1:]))
            checks['config_and_binary_hash']=all(len(events[0]['effective_config_sha256'])==64 for _ in [0]) and len(json.loads((Path(logdir)/'manifest.json').read_text())['binary_sha256'])==64
            checks['log_run_end']=any(e['event_type']=='RUN_END' for e in events)
            record={'name':name,'passed':all(checks.values()),'checks':checks,'expected':expected,'triggered_rules':sorted(rules),'diagnostic_samples':len(snapshots),'log_directory':logdir,'fault_bag_sha256':case['bag_sha256']}
            (out/(name+'_diagnostics.json')).write_text(json.dumps(snapshots,ensure_ascii=False,indent=2)+'\n');results.append(record);print(json.dumps(record,ensure_ascii=False),flush=True)
        finally:stop(player);stop(monitor)
    summary={'source_unchanged':manifest['source_sha256_before']==manifest['source_sha256_after'],'results':results,'passed':sum(r['passed'] for r in results),'total':len(results)}
    (out/'fault_bag_results.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    assert summary['passed']==summary['total'],'fault bag tests failed'
if __name__=='__main__':main()
