#!/usr/bin/env python3
"""Generate isolated fault copies; the original ROS bag is opened read-only."""
import argparse,copy,hashlib,json,math
from pathlib import Path
import rosbag,rospy
from nav_msgs.msg import Odometry

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',default='/home/lmw/datasets/fastlio/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag');ap.add_argument('--output',default='/home/lmw/datasets/fastlio/health_fault_cases');ap.add_argument('--duration',type=float,default=12)
    a=ap.parse_args();source=Path(a.source).resolve();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
    assert source.parent!=out,'Use a separate fault_cases directory'
    original_hash=sha(source);records=[]
    with rosbag.Bag(str(source),'r') as bag:
        begin=bag.get_start_time()
        for topic,msg,t in bag.read_messages():
            if t.to_sec()>begin+a.duration:break
            records.append((topic,msg,t))
    imu_first=next(m for topic,m,t in records if topic=='/livox/imu');stamp0=imu_first.header.stamp.to_sec()
    odoms=[]
    for i in range(int(a.duration*10)):
        e=i*.1;msg=Odometry();msg.header.stamp=rospy.Time.from_sec(stamp0+e);msg.header.frame_id='camera_init';msg.child_frame_id='body';msg.pose.pose.position.x=e*.5;msg.pose.pose.orientation.w=1
        for k in (0,7,14,21,28,35):msg.pose.covariance[k]=.01
        odoms.append(('/health_test/odom',msg,rospy.Time.from_sec(begin+e)))
    combined=sorted(records+odoms,key=lambda x:x[2].to_sec())
    cases={
      'baseline_short':('baseline',None),
      'imu_nan_single':('IMU_ACCEL_FINITE', '/livox/imu'),
      'imu_overlimit':('IMU_ACCEL_NORM','/livox/imu'),
      'imu_unavailable':('IMU_ACCEL_AVAILABLE','/livox/imu'),
      'imu_bad_covariance':('IMU_ACCEL_COV_INVALID','/livox/imu'),
      'imu_stamp_regression':('STAMP_REGRESSION','/livox/imu'),
      'lidar_empty':('LIDAR_EMPTY','/livox/lidar'),
      'lidar_count_mismatch':('LIDAR_STRUCTURE','/livox/lidar'),
      'lidar_dropout':('RECEIVE_AGE','/livox/lidar'),
      'lidar_low_frequency':('FREQUENCY_LOW','/livox/lidar'),
      'localization_jump':('LOC_SPEED','/health_test/odom'),
      'localization_covariance':('LOC_COVARIANCE_INVALID','/health_test/odom'),
      'quaternion_sign_valid':('must_not_trigger_rotation_jump','/health_test/odom'),
    }
    manifest={'source':str(source),'source_sha256_before':original_hash,'duration_sec':a.duration,'bag_start_time_sec':begin,'synthetic_odometry':{'topic':'/health_test/odom','purpose':'deterministic health-rule fixture; not localization ground truth','speed_mps':.5,'rate_hz':10},'cases':[]}
    for name,(expected,target) in cases.items():
        dest=out/(name+'.bag');assert dest!=source;mutations=0;indices=0
        with rosbag.Bag(str(dest),'w',compression=rosbag.Compression.LZ4) as bag:
            for topic,original,t in combined:
                e=t.to_sec()-begin;msg=original
                mutate=topic==target and 5<=e<6
                if name in ('imu_nan_single','imu_stamp_regression','lidar_count_mismatch'):mutate=mutate and mutations==0
                if name=='lidar_dropout' and topic==target and 5<=e<8:mutations+=1;continue
                if name=='lidar_low_frequency' and topic==target and 4<=e<10:
                    indices+=1
                    if indices%5:mutations+=1;continue
                if mutate:
                    msg=copy.deepcopy(original)
                    if name=='imu_nan_single':msg.linear_acceleration.x=float('nan')
                    elif name=='imu_overlimit':msg.linear_acceleration.x=30
                    elif name=='imu_unavailable':msg.linear_acceleration_covariance=list(msg.linear_acceleration_covariance);msg.linear_acceleration_covariance[0]=-1
                    elif name=='imu_bad_covariance':msg.linear_acceleration_covariance=[1,2,0,0,1,0,0,0,1]
                    elif name=='imu_stamp_regression':msg.header.stamp=rospy.Time.from_sec(3)
                    elif name=='lidar_empty':msg.points=[];msg.point_num=0
                    elif name=='lidar_count_mismatch':msg.point_num+=7
                    elif name=='localization_jump':msg.pose.pose.position.x+=30
                    elif name=='localization_covariance':msg.pose.covariance[1]=10
                    elif name=='quaternion_sign_valid':msg.pose.pose.orientation.w=-1
                    else:mutate=False
                    if mutate:mutations+=1
                bag.write(topic,msg,t)
        manifest['cases'].append({'name':name,'bag':str(dest),'bag_sha256':sha(dest),'expected_rule':expected,'mutated_or_dropped_messages':mutations,'fault_interval_sec':[4,10] if name=='lidar_low_frequency' else [5,8] if name=='lidar_dropout' else [5,6]})
    manifest['source_sha256_after']=sha(source);assert manifest['source_sha256_after']==original_hash,'Original was modified'
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'cases':len(cases),'source_unchanged':True,'output':str(out)},ensure_ascii=False))
if __name__=='__main__':main()
