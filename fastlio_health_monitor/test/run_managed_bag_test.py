#!/usr/bin/env python3
"""Run actual FAST-LIO, STOP for >=30s, RUN a new instance, then unexpected loss."""
import json,os,signal,subprocess,time,re
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QGuiApplication
from pathlib import Path
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2
from fastlio_health_monitor.srv import ControlModule
REPO_ROOT=Path(__file__).resolve().parents[2]
OUT=REPO_ROOT/'FAST_LIO/doc/ros1_workspace/evidence/v2'
app=QApplication([])
def screenshot(name):
    tree=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True).stdout
    lines=[l for l in tree.splitlines() if 'RViz' in l and 'health' in l]
    if not lines:return False
    window=int(re.search(r'0x[0-9a-f]+',lines[0]).group(),16)
    return QGuiApplication.primaryScreen().grabWindow(window).save(str(OUT.parent.parent/'images'/name))
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def main():
    OUT.mkdir(exist_ok=True,parents=True);rospy.init_node('v2_managed_acceptance',anonymous=True,disable_signals=True);samples=[];mapstats=[]
    def diag(msg):samples.append({'received':time.monotonic(),'statuses':[{'id':s.hardware_id,'level':int(s.level),'message':s.message,'values':{v.key:v.value for v in s.values}} for s in msg.status]})
    def cloud(msg):mapstats.append({'received':time.monotonic(),'points':msg.width*msg.height,'frame':msg.header.frame_id})
    sub=rospy.Subscriber('/fastlio_health_monitor/diagnostics',DiagnosticArray,diag,queue_size=200);maps=rospy.Subscriber('/Laser_map',PointCloud2,cloud,queue_size=1)
    log=(OUT/'managed_bag_launch.log').open('w');p=subprocess.Popen(['roslaunch','fastlio_health_monitor','managed_bag_testbench.launch','rviz:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);results={};marks={}
    def latest(id):
        for snap in reversed(samples):
            for s in snap['statuses']:
                if s['id']==id:return s
        return None
    def wait(pred,timeout,label):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and p.poll() is None:
            if pred():return
            time.sleep(.1)
        raise AssertionError(label+' timed out; last='+str(latest('localization')))
    try:
        rospy.wait_for_service('/robot_runtime/control',timeout=12);control=rospy.ServiceProxy('/robot_runtime/control',ControlModule)
        wait(lambda:latest('localization') and latest('localization')['values'].get('operating_state')=='RUNNING',25,'first ready')
        marks['initial_ready']=time.monotonic();old=latest('localization')['values']['producer_instance_id'];results['initial_running']=True
        time.sleep(3)
        screenshot('v2_running_20261012.png')
        answer=control('localization','STOP');assert answer.accepted
        wait(lambda:latest('localization')['values'].get('operating_state')=='STOPPED',10,'stopped confirmation');marks['stopped']=time.monotonic()
        deadline=marks['stopped']+30.5
        while time.monotonic()<deadline:time.sleep(.1)
        marks['stop_wait_end']=time.monotonic();stopped=[s for snap in samples if marks['stopped']+.5<=snap['received']<=marks['stop_wait_end'] for s in snap['statuses'] if s['id']=='localization']
        results['stopped_at_least_30_seconds']=marks['stop_wait_end']-marks['stopped']>=30
        results['no_false_error_during_stop']=bool(stopped) and all(s['level']==1 and s['values'].get('operating_state')=='STOPPED' and s['values'].get('operational_valid')=='false' for s in stopped)
        results['sensors_keep_running']=any(s['id']=='imu' and s['values'].get('operating_state')=='RUNNING' for snap in samples if marks['stopped']+5<snap['received']<marks['stop_wait_end'] for s in snap['statuses'])
        screenshot('v2_stopped_20261012.png')
        answer=control('localization','RUN');assert answer.accepted;marks['restart_request']=time.monotonic()
        wait(lambda:latest('localization')['values'].get('operating_state')=='RUNNING',20,'restart ready');marks['restart_ready']=time.monotonic();new=latest('localization')['values']['producer_instance_id'];results['new_instance']=new!=old
        restarted=[s for snap in samples if marks['restart_request']+.5<snap['received']<marks['restart_ready']+1 for s in snap['statuses'] if s['id']=='localization']
        results['no_cross_instance_jump']=not any('LOC_JUMP:' in s['message'] or 'LOC_SPEED:' in s['message'] for s in restarted)
        time.sleep(3)
        screenshot('v2_restarted_20261012.png')
        # Kill only the managed localization node on this isolated ROS master.
        subprocess.run(['rosnode','kill','/laserMapping'],check=True,capture_output=True);marks['unexpected_stop']=time.monotonic()
        wait(lambda:latest('localization')['level']==2,8,'unexpected stop ERROR');results['unexpected_stop_error']=True
        # Stop the manager deliberately; monitor must reject its old STOP state.
        subprocess.run(['rosnode','kill','/robot_runtime_controller'],check=True,capture_output=True);marks['controller_loss']=time.monotonic()
        wait(lambda:latest('localization')['values'].get('operating_state')=='UNKNOWN',12,'controller loss UNKNOWN');results['controller_loss_unknown']=True
        results['map_published']=bool(mapstats) and max(m['points'] for m in mapstats)>1000
        results['log_files_present']=all(Path(latest('monitor')['values']['log_directory'],f).exists() for f in ('manifest.json','effective_config.yaml','events.0.jsonl','metrics.0.csv'))
        subprocess.run(['rosnode','kill','/fastlio_health_monitor'],check=True,capture_output=True);time.sleep(2.5);results['panel_monitor_loss_screenshot']=screenshot('v2_monitor_lost_20261012.png')
    finally:
        stop(p);log.close()
        result={'checks':results,'passed':all(results.values()),'marks':marks,'diagnostic_samples':len(samples),'maps':mapstats,'monitor_log_directory':latest('monitor')['values']['log_directory'] if latest('monitor') else None}
        (OUT/'managed_bag_results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');(OUT/'managed_bag_diagnostics.json').write_text(json.dumps(samples,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False),flush=True)
    assert all(results.values()),'managed acceptance failed'
if __name__=='__main__':main()
