#!/usr/bin/env python3
"""V2 可选规则验收：创建独立错误 bag，启动真实 C++ 节点，检查诊断和日志。

本脚本只负责测试数据和进程编排，不实现健康判断。
数据流：生成 ROS 标准消息 -> 独立 bag -> C++ 回放器 -> C++ 适配器/规则 -> 诊断/日志。
不会改写用户的原始 bag。每次运行请提供一个新的 --output 目录。
"""
import argparse
import csv
import json
import math
import os
import signal
import socket
import struct
import subprocess
import time
from pathlib import Path

import rosbag
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField, Range
from std_srvs.srv import SetBool

FORBIDDEN = {'producer_instance_id', 'module_epoch', 'metric', 'effective_config_sha256', 'data_sha256', 'epoch'}


def stop(process):
    """只关闭本次测试创建的进程组，先 SIGINT，让日志有时间正常封存。"""
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def write_json(path, data):
    """保留可追溯的测试结果；失败时也能看到已经完成的检查。"""
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def forbidden_keys(data):
    """递归查键，不把 reason 等普通字符串中出现的词当作字段。"""
    found = set()
    if isinstance(data, dict):
        found.update(FORBIDDEN.intersection(data))
        for value in data.values():
            found.update(forbidden_keys(value))
    elif isinstance(data, list):
        for value in data:
            found.update(forbidden_keys(value))
    return found


def generate_bag(path):
    """产生 15 秒的五模块样例：断流、倾角、点云空帧、NaN、测距跳变。"""
    begin = 1792396800.0
    with rosbag.Bag(str(path), 'w') as bag:
        for i in range(1501):
            t = i / 100.0
            stamp = rospy.Time.from_sec(begin + t)
            raw = Imu()
            raw.header.stamp = stamp
            raw.header.frame_id = 'base_link'
            raw.linear_acceleration.z = float('nan') if 12 <= t < 12.1 else 9.80665
            raw.orientation_covariance[0] = -1
            if not 6 <= t < 6.8:
                bag.write('/mavros/imu/data_raw', raw, stamp)
            attitude = Imu()
            attitude.header = raw.header
            attitude.linear_acceleration.z = 9.80665
            roll = math.radians(60 if 8 <= t < 8.5 else 0)
            attitude.orientation.x = math.sin(roll / 2)
            attitude.orientation.w = math.cos(roll / 2)
            for j in (0, 4, 8):
                attitude.orientation_covariance[j] = .01
            bag.write('/mavros/imu/data', attitude, stamp)
            if i % 10 == 0:
                cloud = PointCloud2()
                cloud.header.stamp = stamp
                cloud.header.frame_id = 'camera_init'
                cloud.height = 1
                cloud.width = 0 if 11 <= t < 11.3 else 200
                cloud.fields = [PointField(name=n, offset=j * 4, datatype=7, count=1)
                                for j, n in enumerate(('x', 'y', 'z', 'intensity'))]
                cloud.point_step = 16
                cloud.row_step = cloud.width * 16
                cloud.data = b''.join(struct.pack('<ffff', 1+j*.01, math.sin(j*.1), 0., float(j))
                                      for j in range(cloud.width))
                bag.write('/rslidar_points', cloud, stamp)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'camera_init'
                odom.child_frame_id = 'body'
                odom.pose.pose.position.x = t
                pitch = math.radians(50 if 9 <= t < 9.4 else 0)
                odom.pose.pose.orientation.y = math.sin(pitch / 2)
                odom.pose.pose.orientation.w = math.cos(pitch / 2)
                for j in (0, 7, 14, 21, 28, 35):
                    odom.pose.covariance[j] = .01
                bag.write('/Odometry', odom, stamp)
            if i % 33 == 0 and 3 <= t < 13:
                distance = Range()
                distance.header.stamp = stamp
                distance.header.frame_id = 'rangefinder'
                distance.radiation_type = Range.INFRARED
                distance.min_range = 0
                distance.max_range = 100
                distance.range = 8 if 10 <= t < 10.4 else 1
                bag.write('/mavros/rangefinder/rangefinder', distance, stamp)
    return begin


def screenshot(path):
    """截取真实桌面显示，失败则明确记录原因，不生成替代图片。"""
    try:
        import gi
        gi.require_version('Gdk', '3.0')
        from gi.repository import Gdk
        root = Gdk.get_default_root_window()
        pixbuf = Gdk.pixbuf_get_from_window(root, 0, 0, root.get_width(), root.get_height())
        pixbuf.savev(str(path), 'png', [], [])
        return str(path)
    except Exception as exc:
        return 'unavailable: ' + str(exc)


def main():
    """选择配置 -> 建立隔离 ROS master -> 回放并检查 -> 关闭本次进程 -> 写报告。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--package', required=True)
    parser.add_argument('--port', type=int, default=33529)
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--disabled-rules', action='store_true', help='同一个错误bag，用enabled:false关闭可选规则后确认不报警')
    parser.add_argument('--real-bag', help='可选：只读完整回放指定真实 bag，不产生故障注入')
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    package = Path(args.package)
    # 避免占用用户已有 master：目标端口不是空闲状态时立即退出。
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', args.port))
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(args.port)
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    master_log = (out / 'master.log').open('w')
    master = subprocess.Popen(['roscore', '-p', str(args.port)], stdout=master_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    player = None
    handle = None
    checks = {}
    snapshots = []
    latest = {}
    failure = None
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                rospy.get_master().getPid()
                break
            except Exception:
                time.sleep(.1)
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('optional_rules_acceptance', anonymous=True, disable_signals=True)
        bag_path = Path(args.real_bag) if args.real_bag else out / 'synthetic_faults.bag'
        if not args.real_bag:
            generate_bag(bag_path)
        with rosbag.Bag(str(bag_path), 'r') as metadata:
            bag_begin = metadata.get_start_time()
        root = yaml.safe_load((package / 'config/health_bag.yaml').read_text())
        root['logging']['directory'] = str(out / 'health_logs')
        root['monitor'].update(publish_rate_hz=100., soft_warning_hold_sec=0., recovery_hold_sec=.1)
        if not args.real_bag:
            root['monitor'].update(min_frequency_samples=2, min_frequency_coverage_sec=.1)
            root['lifecycle_defaults']['ready_consecutive_samples'] = 2
            for module in root['modules']:
                module['lifecycle']['ready_consecutive_samples'] = 2
                # 合成 IMU 为100Hz：仅调整对应频率，真实传感器配置保持原样。
                if module['checker'] == 'ImuChecker':
                    module['stream']['frequency'] = {'enabled': True, 'warning_below_hz': 80., 'error_below_hz': 20.,
                                                     'warning_above_hz': 130., 'error_above_hz': 200.}
        if args.disabled_rules:
            # 模拟用户日常改法：保留配置和阈值，只关闭明确的规则开关。
            for module in root['modules']:
                for group in ('stream', 'quality'):
                    for rule in module.get(group, {}).values():
                        if isinstance(rule, dict) and 'enabled' in rule:
                            rule['enabled'] = False
        config = out / 'test_config.yaml'
        config.write_text(yaml.safe_dump(root, allow_unicode=True, sort_keys=False))

        def callback(msg):
            # 缓存所有最新模块；证据只取20Hz，避免100Hz诊断占据大量磁盘。
            nonlocal latest
            latest = {s.hardware_id: {'level': int(s.level), 'message': s.message,
                       'values': {v.key: v.value for v in s.values}} for s in msg.status}
            now = time.monotonic()
            if not snapshots or now - snapshots[-1]['steady_observed'] >= .05:
                snapshots.append({'steady_observed': now, 'statuses': latest})

        sub = rospy.Subscriber('/fastlio_health_monitor/diagnostics', DiagnosticArray, callback, queue_size=200)
        handle = (out / 'launch.log').open('w')
        # 基础入口可传日志目录；开启required_inputs_from_config，关闭的模块不再阻塞启动。
        command = ['roslaunch', 'fastlio_health_monitor', 'recorded_bag_health.launch',
                   'bag:=' + str(bag_path), 'health_config:=' + str(config),
                   'required_inputs_from_config:=true', 'rviz:=' + ('true' if args.rviz else 'false'),
                   'playback_log_directory:=' + str(out / 'playback_logs')]
        player = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        started = time.monotonic()
        paused = False
        captured = False
        end_limit = 380 if args.real_bag else 55
        while time.monotonic() - started < end_limit:
            if player.poll() is not None:
                raise RuntimeError('roslaunch exited early: ' + str(player.returncode))
            elapsed = rospy.get_time() - bag_begin
            if latest and args.rviz and not captured and 1.5 < elapsed < 3:
                write_json(out / 'screenshot.json', {'result': screenshot(out / 'optional_no_data.png')})
                captured = True
            if not args.real_bag and not paused and elapsed >= 4:
                rospy.wait_for_service('/recorded_bag_playback/pause', timeout=5)
                service = rospy.ServiceProxy('/recorded_bag_playback/pause', SetBool)
                response = service(True)
                time.sleep(.4)
                states = [m['values'].get('operating_state') for k, m in latest.items()
                          if k not in ('monitor', 'logger')]
                checks['explicit_pause_acknowledged'] = response.success
                checks['all_modules_paused'] = bool(states) and all(s == 'PAUSED' for s in states)
                checks['pause_does_not_report_error'] = all(m['level'] < 2 for m in latest.values())
                checks['resume_acknowledged'] = service(False).success
                paused = True
            if latest.get('monitor', {}).get('values', {}).get('log_recording_state') == 'FINISHED':
                break
            time.sleep(.02)
        else:
            raise RuntimeError('timed out waiting for verified EOF log seal')
        logdir = Path(latest['monitor']['values']['log_directory'])
        events = [json.loads(line) for path in sorted(logdir.glob('events.*.jsonl'))
                  for line in path.read_text().splitlines()]
        triggered = [e for e in events if e['event_type'] == 'RULE_TRIGGERED']
        rules = {(e['module_id'], e['rule_id']) for e in triggered}
        checks['verified_eof_stopped'] = latest['monitor']['values'].get('operating_state') == 'STOPPED'
        checks['single_run_end'] = sum(e['event_type'] == 'RUN_END' for e in events) == 1
        checks['event_sequence_increasing'] = all(a['event_seq'] < b['event_seq'] for a, b in zip(events, events[1:]))
        checks['event_and_context_fields_removed'] = not forbidden_keys(events)
        checks['manifest_fields_removed'] = not forbidden_keys(json.loads((logdir / 'manifest.json').read_text()))
        metric_rows = []
        for path in logdir.glob('metrics.*.csv'):
            with path.open() as stream:
                reader = csv.DictReader(stream)
                checks['csv_columns_removed'] = not FORBIDDEN.intersection(reader.fieldnames)
                for row in reader:
                    checks['csv_nested_fields_removed'] = not forbidden_keys(json.loads(row['values_json'])) and checks.get('csv_nested_fields_removed', True)
                    metric_rows.append(row)
        checks['only_three_record_times'] = all(all(key not in e for key in ('bag_time_sec', 'wall_time_sec', 'ros_time_sec', 'source_stamp_ns')) for e in events)
        before = {str(p): (p.stat().st_size, p.stat().st_mtime_ns) for p in logdir.glob('*') if p.suffix in ('.jsonl', '.csv')}
        count = len(snapshots)
        time.sleep(1.2)
        checks['files_unchanged_after_eof'] = before == {str(p): (p.stat().st_size, p.stat().st_mtime_ns) for p in logdir.glob('*') if p.suffix in ('.jsonl', '.csv')}
        checks['diagnostics_continue_after_eof'] = len(snapshots) > count
        if not args.real_bag:
            checks['pause_exercised'] = paused
            checks['optional_range_initial_no_data'] = any(s['statuses'].get('ground_range', {}).get('values', {}).get('operating_state') == 'NO_DATA' for s in snapshots)
            checks['optional_range_auto_connected'] = any(s['statuses'].get('ground_range', {}).get('values', {}).get('range_status') == 'AVAILABLE' for s in snapshots)
            expected = [('imu', 'RECEIVE_AGE'), ('imu', 'STAMP_GAP'),
                ('imu_attitude', 'IMU_ROLL_LIMIT'), ('localization', 'LOC_PITCH_LIMIT'),
                ('ground_range', 'RANGE_JUMP'), ('lidar', 'LIDAR_EMPTY')]
            for module, rule in expected:
                checks[module + '_' + rule + ('_disabled' if args.disabled_rules else '')] = ((module, rule) not in rules) if args.disabled_rules else ((module, rule) in rules)
            checks['nonfinite_guard_retained'] = ('imu', 'IMU_ACCEL_FINITE') in rules
            if not args.disabled_rules:
                warn = [e for e in triggered if e['module_id'] == 'imu' and e['rule_id'] == 'RECEIVE_AGE' and e['to_level'] == 'WARNING']
                error = [e for e in triggered if e['module_id'] == 'imu' and e['rule_id'] == 'RECEIVE_AGE' and e['to_level'] == 'ERROR']
                checks['watchdog_warn_before_gap_end'] = any(6 <= e['bag_elapsed_sec'] < 6.3 for e in warn)
                checks['watchdog_error_before_gap_end'] = any(6.1 <= e['bag_elapsed_sec'] < 6.5 for e in error)
        else:
            checks['real_range_remains_no_data'] = any(s['statuses'].get('ground_range', {}).get('values', {}).get('operating_state') == 'NO_DATA' for s in snapshots)
            for module in ('lidar', 'imu', 'imu_attitude', 'localization'):
                checks[module + '_reached_running'] = any(s['statuses'].get(module, {}).get('values', {}).get('operating_state') == 'RUNNING' for s in snapshots)
        write_json(out / 'log_summary.json', {'health_log': str(logdir), 'event_count': len(events), 'metric_rows': len(metric_rows),
                     'triggered_rules': sorted([list(item) for item in rules]), 'watchdog_events': [e for e in triggered if e['rule_id'] == 'RECEIVE_AGE']})
        write_json(out / 'diagnostics.json', snapshots)
    except Exception as exc:
        failure = str(exc)
    finally:
        stop(player)
        stop(master)
        if handle:
            handle.close()
        master_log.close()
        write_json(out / 'results.json', {'checks': checks, 'passed': bool(checks) and all(checks.values()) and failure is None, 'failure': failure})
        print(json.dumps({'checks': checks, 'failure': failure}, ensure_ascii=False), flush=True)
    if failure or not checks or not all(checks.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
