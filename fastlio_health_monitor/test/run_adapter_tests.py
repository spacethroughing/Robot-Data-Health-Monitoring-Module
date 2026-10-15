#!/usr/bin/env python3
"""Exercise real C++ adapters, SI units, structured clouds and non-localization modules."""
import copy,json,os,signal,struct,subprocess,threading,time
from pathlib import Path
import rospy,yaml
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
from sensor_msgs.msg import PointCloud2,PointField,Imu
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import PoseStamped
from fastlio_health_monitor.msg import TaskState
OUT=Path('/home/lmw/fastlio_ws/src/FAST_LIO/doc/ros1_workspace/evidence/v2')
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def main():
    rospy.init_node('v2_adapter_acceptance',anonymous=True,disable_signals=True);OUT.mkdir(exist_ok=True,parents=True)
    root=yaml.safe_load(Path('/home/lmw/fastlio_ws/src/fastlio_health_monitor/config/health_v2_pointcloud2.yaml').read_text());ext=yaml.safe_load(Path('/home/lmw/fastlio_ws/src/fastlio_health_monitor/config/health_v2_extensions.yaml').read_text())
    root['modules'][2]['id']='secondary_localizer';root['modules'][2]['topic']='/alt_odom';root['modules'][2].pop('depends_on');root['modules']+=ext['modules'];root['monitor']['publish_rate_hz']=20;root['monitor']['soft_warning_hold_sec']=.05;root['monitor']['recovery_hold_sec']=.2;root['monitor']['min_frequency_coverage_sec']=.3;root['lifecycle_defaults']['ready_consecutive_samples']=3;root['logging']['directory']=str(OUT/'adapter_runtime_logs')
    for m in root['modules']:
        if m['checker']=='PlanningChecker':m['quality']['result_deadline_sec']=.5;m['quality']['task_state_timeout_sec']=.5
    cfg=OUT/'adapter_test.yaml';cfg.write_text(yaml.safe_dump(root,allow_unicode=True,sort_keys=False));latest={};snapshots=[]
    def diag(msg):
        for s in msg.status:latest[s.hardware_id]={'level':int(s.level),'message':s.message,'values':{v.key:v.value for v in s.values}}
        snapshots.append({'received':time.monotonic(),'modules':copy.deepcopy(latest)})
    sub=rospy.Subscriber('/fastlio_health_monitor/diagnostics',DiagnosticArray,diag,queue_size=200)
    pubs={name:rospy.Publisher(topic,typ,queue_size=20) for name,topic,typ in [('cloud','/points_raw',PointCloud2),('imu','/imu/data',Imu),('odom','/alt_odom',Odometry),('relay','/perception/diagnostics',DiagnosticArray),('path','/planner/path',RosPath),('task','/planner/task_state',TaskState)]}
    flags={'malformed':False,'big_endian':False,'imu_cov':False,'imu_missing':False,'relay_empty':False,'relay_invalid':False,'relay_negative':False,'goal_invalid':False,'goal':False,'path':False,'task':True};done=threading.Event();results={}
    def publish():
        i=0
        while not done.is_set():
            i+=1;stamp=rospy.Time.from_sec(100+i*.1)
            c=PointCloud2();c.header.stamp=stamp;c.header.frame_id='lidar';c.height=2;c.width=100;c.point_step=24;c.row_step=100*24+8;c.is_bigendian=flags['big_endian'];c.fields=[PointField(name=n,offset=j*8,datatype=8,count=1) for j,n in enumerate(('x','y','z'))];row=b''.join(struct.pack('>ddd' if c.is_bigendian else '<ddd',1+j*.01,2,3) for j in range(100))+bytes(8);c.data=row*2
            if flags['malformed']:c.width=4294967295
            pubs['cloud'].publish(c)
            imu=Imu();imu.header.stamp=rospy.Time.from_sec(100+i*.005);imu.linear_acceleration.z=9.80665;imu.orientation_covariance[0]=-1
            if flags['imu_cov']:imu.linear_acceleration_covariance=[1,2,0,2,1,0,0,0,1]
            if flags['imu_missing']:imu.linear_acceleration_covariance[0]=-1
            pubs['imu'].publish(imu)
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.orientation.w=1
            for j in (0,7,14,21,28,35):o.pose.covariance[j]=.01
            pubs['odom'].publish(o)
            d=DiagnosticArray();d.header.stamp=stamp
            if not flags['relay_empty']:d.status=[DiagnosticStatus(level=-1 if flags['relay_negative'] else 127 if flags['relay_invalid'] else 0,name='objects',message='zero detections is valid')]
            pubs['relay'].publish(d)
            if flags['task']:
                t=TaskState();t.header.stamp=stamp;t.goal_id='goal-A' if flags['goal'] and not flags['goal_invalid'] else '';t.active=flags['goal'];t.state='ACTIVE' if flags['goal'] else 'IDLE';pubs['task'].publish(t)
            if flags['path']:
                path=RosPath();path.header.stamp=stamp;path.header.frame_id='map'
                for j in range(3):p=PoseStamped();p.header=path.header;p.pose.position.x=j*.2;p.pose.orientation.w=1;path.poses.append(p)
                pubs['path'].publish(path)
            done.wait(.05)
    def wait(pred,label,timeout=7):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            if pred():results[label]=True;return
            if node.poll() is not None:break
            time.sleep(.05)
        results[label]=False;raise AssertionError(label+' failed; latest='+str(latest))
    log=(OUT/'adapter_node.log').open('w');node=subprocess.Popen(['rosrun','fastlio_health_monitor','robot_health_monitor_node','_config_file:='+str(cfg),'__name:=v2_adapter_monitor'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);thread=threading.Thread(target=publish);thread.start()
    try:
        wait(lambda:latest.get('secondary_localizer',{}).get('level')==0 and latest.get('secondary_localizer',{}).get('values',{}).get('operating_state')=='RUNNING','second_localizer_generic_id')
        wait(lambda:latest.get('lidar',{}).get('values',{}).get('points_valid')=='200' and latest['lidar']['level']==0,'organized_cloud_float64_padding')
        wait(lambda:latest.get('imu',{}).get('level')==0 and abs(float(latest['imu']['values']['linear_acceleration_norm_mps2'])-9.80665)<1e-8 and latest['imu']['values']['imu_acceleration_scale']=='1','si_imu_not_rescaled')
        wait(lambda:latest.get('planning',{}).get('values',{}).get('operating_state')=='IDLE' and latest['planning']['level']==0,'planning_idle_without_path')
        flags['big_endian']=True;time.sleep(.3);wait(lambda:latest['lidar']['values'].get('points_valid')=='200' and latest['lidar']['level']==0,'big_endian_cloud')
        flags['malformed']=True;wait(lambda:'LIDAR_STRUCTURE' in latest['lidar']['message'] and latest['lidar']['level']==2,'overflow_dimensions_rejected');flags['malformed']=False;wait(lambda:latest['lidar']['level']==0,'cloud_recovers')
        flags['imu_cov']=True;wait(lambda:'IMU_ACCEL_COV_INVALID' in latest['imu']['message'] and latest['imu']['level']==2,'non_psd_imu_covariance');flags['imu_cov']=False;wait(lambda:latest['imu']['level']==0,'unknown_covariance_not_zero_noise')
        flags['imu_missing']=True;wait(lambda:'IMU_ACCEL_AVAILABLE' in latest['imu']['message'] and latest['imu']['level']==2,'required_imu_measurement_missing');flags['imu_missing']=False;wait(lambda:latest['imu']['level']==0,'imu_recovers')
        wait(lambda:latest.get('perception',{}).get('level')==0,'valid_empty_scene_diagnostics')
        flags['relay_invalid']=True;wait(lambda:latest['perception']['level']==2 and 'RELAY_LEVEL_VALID' in latest['perception']['message'],'invalid_relay_level_clamped_to_error');flags['relay_invalid']=False;wait(lambda:latest['perception']['level']==0,'invalid_relay_level_recovers')
        flags['relay_negative']=True;wait(lambda:latest['perception']['level']==2 and 'RELAY_LEVEL_VALID' in latest['perception']['message'],'negative_relay_level_rejected');flags['relay_negative']=False;wait(lambda:latest['perception']['level']==0,'negative_relay_level_recovers')
        flags['relay_empty']=True;wait(lambda:'RELAY_NONEMPTY' in latest['perception']['message'] and latest['perception']['level']==2,'empty_diagnostic_array_rejected');flags['relay_empty']=False;wait(lambda:latest['perception']['level']==0,'relay_recovers')
        flags['goal']=True;wait(lambda:'PLANNING_DEADLINE' in latest['planning']['message'] and latest['planning']['level']==2,'active_goal_result_timeout');flags['path']=True;wait(lambda:latest['planning']['level']==0 and latest['planning']['values'].get('operational_valid')=='true','planning_valid_path_recovery')
        flags['goal_invalid']=True;wait(lambda:'PLANNING_GOAL_ID' in latest['planning']['message'] and latest['planning']['level']==2,'active_goal_requires_id');flags['goal_invalid']=False;wait(lambda:latest['planning']['level']==0,'goal_id_recovers')
        flags['task']=False;wait(lambda:'PLANNING_TASK_FRESH' in latest['planning']['message'] and latest['planning']['level']==2,'planning_task_heartbeat_timeout');flags['task']=True;wait(lambda:latest['planning']['level']==0,'planning_task_recovers')
    finally:
        done.set();thread.join();stop(node);log.close();(OUT/'adapter_results.json').write_text(json.dumps({'checks':results,'passed':all(results.values()),'snapshots':len(snapshots)},ensure_ascii=False,indent=2)+'\n');(OUT/'adapter_diagnostics.json').write_text(json.dumps(snapshots,ensure_ascii=False,indent=2)+'\n');print(json.dumps(results,ensure_ascii=False),flush=True)
    assert all(results.values())
if __name__=='__main__':main()
