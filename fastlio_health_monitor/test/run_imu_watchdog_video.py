#!/usr/bin/env python3
"""Record real RViz input-silence tests and measure alerts before the next IMU."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import time
import xmlrpc.client

import gi
import yaml
gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
gi.require_version("GdkX11", "3.0")
gi.require_version("Gst", "1.0")
from gi.repository import Gtk, Gdk, GdkX11, Gst
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Imu

WS = Path("/home/lmw/fastlio_ws")
PACKAGE = WS / "src/fastlio_health_monitor"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1048576), b""):
            h.update(chunk)
    return h.hexdigest()


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--evidence", required=True)
    ap.add_argument("--video", required=True)
    ap.add_argument("--port", type=int, default=33514)
    ap.add_argument("--original-record-bag")
    args = ap.parse_args()
    out, video = Path(args.evidence), Path(args.video)
    if video.exists():
        raise RuntimeError("Refusing to overwrite video")
    out.mkdir(parents=True, exist_ok=True)
    (out / "images").mkdir(exist_ok=True)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", args.port))
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % args.port
    Gst.init(None)
    source_config = PACKAGE / "config/health_v2_avia.yaml"
    config_hash_before = sha256(source_config)
    base = yaml.safe_load(source_config.read_text())
    manifest = json.loads(Path(args.bag).with_suffix(".manifest.json").read_text())
    processes, handles = [], []
    recording = None
    context = {"run": None}
    results = []
    screenshots = set()

    def start(command, name):
        print("$ " + shlex.join(command), flush=True)
        handle = (out / (name + ".log")).open("w")
        handles.append(handle)
        p = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                             start_new_session=True)
        processes.append(p)
        return p

    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            while Gtk.events_pending():
                Gtk.main_iteration_do(False)
            if recording:
                error = recording.get_bus().pop_filtered(Gst.MessageType.ERROR)
                if error:
                    reason, debug = error.parse_error()
                    raise RuntimeError(str(reason) + " " + str(debug))
            time.sleep(.005)

    def screenshot(name):
        pix = Gdk.pixbuf_get_from_window(Gdk.get_default_root_window(), 0, 0, 1920, 1080)
        pix.savev(str(out / "images" / (name + ".png")), "png", [], [])

    css = Gtk.CssProvider()
    css.load_from_data(b"""
        window {background-color:#101827;}
        label {color:#edf3fc;font-family:'Noto Sans CJK SC';font-size:20px;}
        .title {font-size:38px;font-weight:bold;color:#6ee7b7;}
        .heading {font-size:25px;font-weight:bold;color:#6ee7b7;}
        .code {font-family:monospace;font-size:18px;color:#b9d5ef;}
        .small {font-size:19px;color:#b9c9df;}
    """)
    Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css, 600)

    def label(text, style="", wrap=True):
        widget = Gtk.Label(label=text)
        widget.set_xalign(0)
        widget.set_line_wrap(wrap)
        if style:
            widget.get_style_context().add_class(style)
        return widget

    def card(title, body):
        window = Gtk.Window()
        window.set_decorated(False)
        window.set_keep_above(True)
        window.fullscreen()
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=35)
        box.set_margin_start(100)
        box.set_margin_end(100)
        box.set_valign(Gtk.Align.CENTER)
        box.pack_start(label(title, "title"), False, False, 0)
        box.pack_start(label(body), False, False, 0)
        window.add(box)
        return window

    intro = card("IMU 时间探针 / 错误 bag 完整测试",
                 "真实 C++ V2 监控 + rosbag 1倍速 + RViz 实时录屏\n\n"
                 "共同阈值：100 ms WARNING，200 ms ERROR\n"
                 "错误副本：第6～7秒删除IMU；第12～12.15秒短时删除IMU\n"
                 "A/B/C按原始IMU源时间排程；D保留原bag的约100ms批量回放\n"
                 "雷达记录和原始消息时间戳保持不变，原始bag不覆盖\n\n"
                 "A：5 Hz / 消抖0.5秒\nB：100 Hz / 消抖0.5秒\n"
                 "C：100 Hz / 消抖0秒（当前配置）\nD：当前配置 + 原始记录时序\n\n"
                 "验收：下一条IMU到来前是否报出WARNING和ERROR？")
    sidebar = Gtk.Window()
    sidebar.set_decorated(False)
    sidebar.set_keep_above(False)
    sidebar.set_default_size(1920, 1080)
    sidebar.fullscreen()
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
    box.set_size_request(630, -1)
    for method in (box.set_margin_start, box.set_margin_end, box.set_margin_top, box.set_margin_bottom):
        method(18)
    widgets = {}
    for key, text, style in [
        ("title", "IMU断流时间探针", "heading"),
        ("case", "", "heading"), ("config", "", "code"),
        ("phase", "启动监控并等待连接", ""),
        ("state", "等待诊断", "heading"),
        ("metrics", "", ""), ("long", "1秒断流：等待", ""),
        ("short", "150ms断流：等待", ""),
        ("reason", "", "small"), ("commands", "", "code"),
        ("note", "探针计时用单调时钟；不是等待下一帧再算时间差。\n"
         "这是输入监控测试，没有运行FAST-LIO建图。\n"
         "告警延迟以诊断接收时间测量，包含传输与测试观察开销。", "small"),
    ]:
        w = label(text, style)
        w.set_max_width_chars(37)
        widgets[key] = w
        box.pack_start(w, False, False, 0)
    row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
    row.pack_start(box, False, False, 0)
    spacer = Gtk.Box()
    row.pack_start(spacer, True, True, 0)
    sidebar.add(row)

    def diag(msg):
        run = context["run"]
        if run is None:
            return
        now = time.monotonic()
        for status in msg.status:
            values = {v.key: v.value for v in status.values}
            if values.get("module_id") == "imu":
                run["diagnostics"].append({"received_monotonic": now,
                    "ros_time_sec": msg.header.stamp.to_sec(),
                    "level": int(status.level), "message": status.message,
                    "values": values})
            elif values.get("module_id") == "overall":
                run["log_directory"] = values.get("log_directory", "")

    def imu(msg):
        run = context["run"]
        if run is not None:
            run["inputs"].append({"received_monotonic": time.monotonic(),
                                  "source_stamp_sec": msg.header.stamp.to_sec(),
                                  "ros_time_sec": rospy.Time.now().to_sec()})

    def first_alert(records, start, end, level, threshold):
        return next((r for r in records
                     if start < r["received_monotonic"] < end
                     and r["level"] == level
                     and float(r["values"].get("age_sec", "0")) > threshold
                     and ("RECEIVE_AGE" in r["message"] or
                          "RUNTIME_FAULT" in r["message"])), None)

    def gaps(run):
        return [(a, b) for a, b in zip(run["inputs"], run["inputs"][1:])
                if b["received_monotonic"] - a["received_monotonic"] > .08]

    def analyze(run):
        relevant = {}
        for a, b in gaps(run):
            current_manifest = run["bag_manifest"]
            resume = b["ros_time_sec"] - current_manifest["bag_start_time_sec"]
            fault = next((f for f in current_manifest["faults"]
                          if ((f.get("source_stamp_after_sec") is not None and
                               abs(b["source_stamp_sec"]-f["source_stamp_after_sec"]) < 1e-6 and
                               abs(a["source_stamp_sec"]-f["source_stamp_before_sec"]) < 1e-6)
                              or (f.get("source_stamp_after_sec") is None and
                                  abs(resume-f["interval_sec"][1]) < .35))), None)
            if not fault:
                continue
            start, end = a["received_monotonic"], b["received_monotonic"]
            warning = first_alert(run["diagnostics"], start, end, 1, .1)
            error = first_alert(run["diagnostics"], start, end, 2, .2)
            latency = lambda r: (r["received_monotonic"] - start) * 1000 if r else None
            baseline_ok = any(r["values"].get("operating_state") == "RUNNING"
                              and r["level"] == 0
                              and start - 1 < r["received_monotonic"] < start
                              for r in run["diagnostics"])
            item = {"fault_id": fault["id"], "baseline_imu_running_ok": baseline_ok,
                    "last_input_monotonic": start, "next_input_monotonic": end,
                    "observed_gap_ms": (end - start) * 1000,
                    "source_stamp_gap_sec": b["source_stamp_sec"] - a["source_stamp_sec"],
                    "warning_latency_ms": latency(warning), "error_latency_ms": latency(error),
                    "warning_before_next_imu": warning is not None,
                    "error_before_next_imu": error is not None,
                    "warning_timely_150ms": warning is not None and .098 <= latency(warning)/1000 <= .15,
                    "error_timely_250ms": error is not None and .198 <= latency(error)/1000 <= .25}
            if fault["id"] == "short_150ms":
                item["no_receive_error_before_resume"] = error is None
            previous = relevant.get(fault["id"])
            if previous is None or item["observed_gap_ms"] > previous["observed_gap_ms"]:
                relevant[fault["id"]] = item
        return list(relevant.values())

    def update(run, name):
        rows = run["diagnostics"]
        inputs = run["inputs"]
        if not rows:
            return
        latest = rows[-1]
        elapsed = latest["ros_time_sec"] - manifest["bag_start_time_sec"]
        widgets["phase"].set_text("回放进度：%.2f / 18.00 s\n1倍速，画面未加速或剪切" % max(0, elapsed))
        color = ["#6ee7b7", "#ffcf66", "#ff7788"][latest["level"]]
        widgets["state"].set_markup('<span foreground="%s">IMU：%s / %s</span>' %
            (color, ["OK", "WARNING", "ERROR"][latest["level"]],
             latest["values"].get("operating_state", "")))
        widgets["metrics"].set_text("C++接收年龄：%s s\n已收到IMU：%d条\n最新源时间戳：%s" %
            (latest["values"].get("age_sec", "N/A"), len(inputs),
             "%.6f" % inputs[-1]["source_stamp_sec"] if inputs else "N/A"))
        widgets["reason"].set_text(latest["message"][:260])
        for fault, key in [(manifest["faults"][0], "long"), (manifest["faults"][1], "short")]:
            if elapsed < fault["interval_sec"][0] or not inputs:
                continue
            completed = next((item for item in analyze(run) if item["fault_id"] == fault["id"]), None)
            if completed:
                fmt = lambda value: "未报" if value is None else "%.1f ms" % value
                widgets[key].set_text("%s\nWARNING：%s\nERROR：%s（均只统计下一帧前）" %
                    ("1秒断流" if key == "long" else "150ms短断流",
                     fmt(completed["warning_latency_ms"]), fmt(completed["error_latency_ms"])))
            elif fault["interval_sec"][0] <= elapsed < fault["interval_sec"][1]:
                start = inputs[-1]["received_monotonic"]
                warning = first_alert(rows, start, time.monotonic()+.001, 1, .1)
                error = first_alert(rows, start, time.monotonic()+.001, 2, .2)
                fmt = lambda r: "等待" if r is None else "%.1f ms" % ((r["received_monotonic"]-start)*1000)
                widgets[key].set_text("%s：正在断流\nWARNING：%s\nERROR：%s" %
                    ("1秒断流" if key == "long" else "150ms短断流", fmt(warning), fmt(error)))
                if error and key == "long" and name not in screenshots:
                    pump(.025)
                    screenshot(name + "_error_before_next_imu")
                    screenshots.add(name)

    try:
        start(["roscore", "-p", str(args.port)], "master")
        until = time.monotonic() + 15
        while time.monotonic() < until:
            try:
                if xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getPid("/watchdog_test")[0] == 1:
                    break
            except OSError:
                pass
            pump(.1)
        else:
            raise RuntimeError("ROS master failed")
        subprocess.run(["rosparam", "set", "/use_sim_time", "true"], check=True)
        rospy.init_node("imu_watchdog_video_test", disable_signals=True)
        diag_sub = rospy.Subscriber("/fastlio_health_monitor/diagnostics", DiagnosticArray, diag, queue_size=1000)
        imu_sub = rospy.Subscriber("/livox/imu", Imu, imu, queue_size=1000)
        rviz = yaml.safe_load((WS / "src/FAST_LIO/doc/ros1_workspace/evidence/v2/fault_rviz_20261013/fault_rviz.rviz").read_text())
        rviz["Panels"] = [p for p in rviz["Panels"] if p["Class"] == "fastlio_health_monitor/HealthPanel"]
        manager = rviz["Visualization Manager"]
        manager["Displays"] = [d for d in manager["Displays"] if d["Class"] == "rviz/PointCloud2"]
        manager["Global Options"]["Fixed Frame"] = "livox_raw"
        manager["Views"]["Current"]["Target Frame"] = "livox_raw"
        rviz_file = out / "imu_watchdog_test.rviz"
        rviz_file.write_text(yaml.safe_dump(rviz, allow_unicode=True, sort_keys=False))
        start(["rviz", "-d", str(rviz_file), "__name:=imu_watchdog_rviz"], "rviz")
        intro.show_all()
        pump(2)
        recording = Gst.parse_launch(
            'ximagesrc use-damage=false show-pointer=false startx=0 starty=0 endx=1919 endy=1079 '
            '! video/x-raw,framerate=20/1 ! videoconvert ! video/x-raw,format=I420 '
            '! x264enc tune=zerolatency speed-preset=veryfast bitrate=5000 key-int-max=40 '
            '! h264parse ! mp4mux faststart=true ! filesink location="%s"' % video)
        recording.set_state(Gst.State.PLAYING)
        recorded_start = time.monotonic()
        pump(7)
        window_id = None
        until = time.monotonic() + 15
        while time.monotonic() < until:
            tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
            match = next((line for line in tree.splitlines() if "imu_watchdog_test.rviz" in line), None)
            if match:
                window_id = int(re.search(r"0x[0-9a-f]+", match).group(), 16)
                break
            pump(.1)
        if window_id is None:
            raise RuntimeError("RViz window missing")
        foreign = GdkX11.X11Window.foreign_new_for_display(Gdk.Display.get_default(), window_id)
        foreign.unmaximize()
        Gdk.flush()
        intro.hide()
        sidebar.show_all()
        sidebar.present()
        pump(1)
        foreign.set_keep_above(True)
        foreign.move_resize(645, 5, 1260, 1025)
        foreign.show()
        foreign.raise_()
        Gdk.flush()
        pump(1)
        screenshot("layout_ready")
        variants = [("A_5hz_hold500ms", "A：旧调度与消抖", 5.0, .5),
                    ("B_100hz_hold500ms", "B：只提高检查频率", 100.0, .5),
                    ("C_100hz_hold0", "C：当前100Hz / 无额外消抖", 100.0, .0)]
        if args.original_record_bag:
            variants.append(("D_original_record_times", "D：当前配置 / 原时序批量回放", 100.0, .0))
        for name, description, frequency, hold in variants:
            current_bag = args.original_record_bag if name.startswith("D_") else args.bag
            current_manifest = json.loads(Path(current_bag).with_suffix(".manifest.json").read_text())
            config = copy.deepcopy(base)
            config["monitor"]["publish_rate_hz"] = frequency
            config["monitor"]["soft_warning_hold_sec"] = hold
            config["monitor"]["data_reference"] = current_bag
            config["logging"]["directory"] = str(out / "runtime_logs")
            config["modules"] = [m for m in config["modules"] if m["id"] in ("imu", "lidar")]
            for module in config["modules"]:
                module["required"] = module["id"] == "imu"
                if module["id"] == "imu":
                    module["stream"]["receive_age_sec"] = {"warning_above": .1, "error_above": .2}
            cfg = out / (name + ".yaml")
            cfg.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
            run = {"name": name, "frequency_hz": frequency, "soft_hold_sec": hold,
                   "config": str(cfg), "bag_manifest": current_manifest,
                   "inputs": [], "diagnostics": []}
            context["run"] = run
            widgets["case"].set_text(description)
            widgets["config"].set_text("publish_rate_hz: %.0f\nsoft_warning_hold_sec: %.1f\n"
                "IMU receive_age_sec:\n  warning_above: 0.1\n  error_above: 0.2" % (frequency, hold))
            widgets["long"].set_text("1秒断流：等待第6秒")
            widgets["short"].set_text("150ms短断流：等待第12秒")
            widgets["commands"].set_text("实际执行：\nrosrun fastlio_health_monitor\n  robot_health_monitor_node\n  _config_file:=" + cfg.name +
                "\nrosbag play --clock --rate=1\n" + ("  源时间排程副本" if not name.startswith("D_") else "  原记录时序副本"))
            node = start(["rosrun", "fastlio_health_monitor", "robot_health_monitor_node",
                          "__name:=imu_watchdog_monitor", "_config_file:=" + str(cfg)], name + "_node")
            until = time.monotonic() + 8
            while not run["diagnostics"] and time.monotonic() < until:
                pump(.02)
            if not run["diagnostics"]:
                raise RuntimeError("No monitor diagnostics")
            pump(1)
            player = start(["rosbag", "play", "--clock", "--rate=1", "--delay=.8",
                            "--wait-for-subscribers", current_bag], name + "_player")
            until = time.monotonic() + 30
            while player.poll() is None and time.monotonic() < until:
                update(run, name)
                pump(.03)
                if node.poll() is not None:
                    raise RuntimeError("C++ monitor exited unexpectedly")
            if player.poll() != 0:
                raise RuntimeError("Bag playback failed/timed out")
            update(run, name)
            widgets["phase"].set_text("回放结束：EOF断流不参与故障窗口验收")
            screenshot(name + "_complete")
            pump(1)
            stop(node)
            context["run"] = None
            faults = analyze(run)
            logdir = Path(run["log_directory"])
            events = [json.loads(line) for p in sorted(logdir.glob("events.*.jsonl"))
                      for line in p.read_text().splitlines()]
            summary = {key: value for key, value in run.items() if key not in ("inputs", "diagnostics")}
            summary["faults"] = faults
            summary["diagnostic_count"] = len(run["diagnostics"])
            summary["imu_count"] = len(run["inputs"])
            summary["warning_outside_injected_faults"] = sum(
                r["level"] == 1 and "RECEIVE_AGE" in r["message"] and
                2 < r["ros_time_sec"]-current_manifest["bag_start_time_sec"] < 5
                for r in run["diagnostics"])
            summary["has_receive_age_trigger_event"] = any(e["rule_id"] == "RECEIVE_AGE" and
                e["event_type"] == "RULE_TRIGGERED" for e in events)
            (out / (name + "_observations.json")).write_text(json.dumps(run, ensure_ascii=False, indent=2)+"\n")
            results.append(summary)
            print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
            pump(2)
        target = next(r for r in results if r["name"] == "C_100hz_hold0")
        long = next(item for item in target["faults"] if item["fault_id"] == "long_1s")
        short = next(item for item in target["faults"] if item["fault_id"] == "short_150ms")
        checks = {"two_fault_windows_measured": len(target["faults"]) == 2,
                  "long_gap_baseline_running_ok": long["baseline_imu_running_ok"],
                  "long_warning_before_next_imu": long["warning_before_next_imu"],
                  "long_error_before_next_imu": long["error_before_next_imu"],
                  "long_warning_by_150ms": long["warning_timely_150ms"],
                  "long_error_by_250ms": long["error_timely_250ms"],
                  "short_warning_before_next_imu": short["warning_before_next_imu"],
                  "short_no_receive_error_before_resume": short["no_receive_error_before_resume"],
                  "configuration_not_modified": config_hash_before == sha256(source_config),
                  "original_bag_unchanged": manifest["source_sha256_before"] == sha256(manifest["source"]),
                  "fault_bag_matches_manifest": manifest["bag_sha256"] == sha256(args.bag)}
        summary = {"date": "2026-10-14", "type": "real_bag_real_cpp_real_rviz_unedited_screen_recording",
                   "video": str(video), "bag_manifest": manifest,
                   "source_config_sha256": config_hash_before, "master_port": args.port,
                   "latency_policy": "diagnostic observer monotonic time minus last IMU observer receipt; includes observer/transport overhead",
                   "acceptance_limits_ms": {"warning": [98, 150], "error": [198, 250]},
                   "exact_100ms_200ms_guarantee": False, "results": results,
                   "checks": checks, "passed": all(checks.values())}
        (out / "results.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
        text = "\n".join("%s：WARNING %s；ERROR %s" %
               (r["name"].split("_")[0],
                next(("%.1f ms" % f["warning_latency_ms"] if f["warning_latency_ms"] is not None else "未报"
                      for f in r["faults"] if f["fault_id"] == "long_1s"), "N/A"),
                next(("%.1f ms" % f["error_latency_ms"] if f["error_latency_ms"] is not None else "未报"
                      for f in r["faults"] if f["fault_id"] == "long_1s"), "N/A")) for r in results)
        end_card = card("实测结果：" + ("当前配置通过" if summary["passed"] else "存在未通过项"),
            "1秒断流，以下告警均出现在下一条IMU之前：\n\n" + text +
            "\n\n当前配置150ms短断流：WARNING %s，空窗期间接收超时ERROR未报。\n"
            "恢复帧仍会检查源时间戳间隔，可能触发STAMP_GAP，这是另一条规则。\n"
            "D为原始记录时序对照：100ms一批，可能在未注入故障时也触发WARNING。\n\n"
            "测量包含诊断传输开销；100/200ms是阈值，不是硬实时保证。\n"
            "原始bag与运行YAML未修改；配置快照、事件日志、诊断序列已归档。" %
            ("%.1f ms" % short["warning_latency_ms"] if short["warning_latency_ms"] else "未报"))
        foreign.set_keep_above(False)
        foreign.hide()
        end_card.show_all()
        pump(2)
        screenshot("summary")
        pump(10)
        recording.send_event(Gst.Event.new_eos())
        message = recording.get_bus().timed_pop_filtered(15*Gst.SECOND, Gst.MessageType.EOS|Gst.MessageType.ERROR)
        if not message or message.type != Gst.MessageType.EOS:
            raise RuntimeError("MP4 finalization failed")
        recording.set_state(Gst.State.NULL)
        recording = None
        summary["recording_duration_sec"] = time.monotonic() - recorded_start
        summary["video_sha256"] = sha256(video)
        (out / "results.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
        print("COMPLETE " + str(video) + " passed=" + str(summary["passed"]), flush=True)
        if not summary["passed"]:
            raise AssertionError(str(checks))
    finally:
        context["run"] = None
        if recording:
            recording.send_event(Gst.Event.new_eos())
            recording.get_bus().timed_pop_filtered(5*Gst.SECOND, Gst.MessageType.EOS|Gst.MessageType.ERROR)
            recording.set_state(Gst.State.NULL)
        for process in reversed(processes):
            stop(process)
        rospy.signal_shutdown("watchdog video complete")
        sidebar.destroy()
        intro.destroy()
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    main()
