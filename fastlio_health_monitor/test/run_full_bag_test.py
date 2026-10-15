#!/usr/bin/env python3
"""Full original bag regression with mapping, trace logs and process resource samples."""
import json,os,signal,subprocess,time,xmlrpc.client
from pathlib import Path
import rospy,rosbag
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2
OUT=Path('/home/lmw/fastlio_ws/src/FAST_LIO/doc/ros1_workspace/evidence/v2')
BAG='/home/lmw/datasets/fastlio/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag'
def main():
    rospy.init_node('v2_full_bag_regression',anonymous=True,disable_signals=True);counts={'/livox/lidar':0,'/livox/imu':0};diagnostics=[];maps=[];resources=[]
    def raw(msg,topic):counts[topic]+=1
    subs=[rospy.Subscriber(topic,rospy.AnyMsg,raw,callback_args=topic,queue_size=2000) for topic in counts]
    def diag(msg):diagnostics.append({'received':time.monotonic(),'statuses':[{'id':s.hardware_id,'level':int(s.level),'message':s.message,'values':{v.key:v.value for v in s.values}} for s in msg.status]})
    def cloud(msg):maps.append({'received':time.monotonic(),'points':msg.width*msg.height,'frame':msg.header.frame_id})
    subs+=[rospy.Subscriber('/fastlio_health_monitor/diagnostics',DiagnosticArray,diag,queue_size=500),rospy.Subscriber('/Laser_map',PointCloud2,cloud,queue_size=5)]
    with rosbag.Bag(BAG,'r') as bag:expected={topic:bag.get_message_count(topic_filters=[topic]) for topic in counts}
    log=(OUT/'full_bag_launch.log').open('w');node=subprocess.Popen(['roslaunch','fastlio_health_monitor','visualization_bag_testbench.launch','rviz:=false','rate:=2.0'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    begin=time.monotonic();ended=False
    try:
        while time.monotonic()-begin<95:
            if counts==expected and diagnostics and all(any(s['id']==id and s['level']==2 for s in diagnostics[-1]['statuses']) for id in ('lidar','imu','localization')):ended=True;break
            master=xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11331'))
            try:
                reply=master.lookupNode('full_bag_test','/fastlio_health_monitor')
                if reply[0]==1:
                    pid=xmlrpc.client.ServerProxy(reply[2]).getPid('full_bag_test')[2]
                    text=Path('/proc/'+str(pid)+'/status').read_text();rss=next(int(l.split()[1]) for l in text.splitlines() if l.startswith('VmRSS:'))
                    resources.append({'received':time.monotonic(),'pid':pid,'rss_kb':rss})
            except (OSError,ValueError,xmlrpc.client.Error):pass
            time.sleep(.5)
        if not ended:print('Playback deadline; counts='+str(counts)+', expected='+str(expected),flush=True)
    finally:
        if node.poll() is None:
            os.killpg(node.pid,signal.SIGINT)
            try:node.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(node.pid,signal.SIGKILL);node.wait()
        log.close()
    assert diagnostics,'no diagnostics'
    logdir=next(s['values']['log_directory'] for d in diagnostics for s in d['statuses'] if s['id']=='monitor')
    def statuses(id):return [s for d in diagnostics for s in d['statuses'] if s['id']==id]
    running=[s for s in statuses('localization') if s['values'].get('operating_state')=='RUNNING']
    checks={'playback_completed':ended,'all_raw_inputs_received':counts==expected,'monitor_received_all_inputs':all(int(statuses(id)[-1]['values'].get('message_count','0'))==expected[topic] for id,topic in (('lidar','/livox/lidar'),('imu','/livox/imu'))),'localization_running':bool(running),'map_received':bool(maps) and max(m['points'] for m in maps)>50000,'imu_si_conversion':any(s['values'].get('imu_acceleration_scale')=='9.80665' and s['level']==0 for s in statuses('imu')),'fresh_covariance_available':any(s['values'].get('covariance_status')=='AVAILABLE' and float(s['values'].get('position_variance_m2','100'))>0 for s in running),'eof_reports_stale':all(any(s['level']==2 for s in statuses(id)[-4:]) for id in ('lidar','imu','localization')),'logs_healthy':all(s['values'].get('logs_incomplete')=='false' for s in statuses('monitor'))}
    results={'checks':checks,'passed':all(checks.values()),'counts':counts,'expected':expected,'diagnostic_samples':len(diagnostics),'maps':maps,'log_directory':logdir,'maximum_monitor_rss_kb':max((r['rss_kb'] for r in resources),default=0),'maximum_diagnostic_gap_sec':max((b['received']-a['received'] for a,b in zip(diagnostics,diagnostics[1:])),default=0),'localization_running_levels':{str(l):sum(s['level']==l for s in running) for l in range(3)}}
    (OUT/'full_bag_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n');(OUT/'full_bag_diagnostics.json').write_text(json.dumps(diagnostics,ensure_ascii=False,indent=2)+'\n');(OUT/'full_bag_resources.json').write_text(json.dumps(resources,indent=2)+'\n');print(json.dumps(results,ensure_ascii=False),flush=True);assert all(checks.values())
if __name__=='__main__':main()
