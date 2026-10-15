#!/usr/bin/env python3
"""Create an IMU silence fixture while preserving the original Livox bag."""
import argparse
import hashlib
import json
from pathlib import Path

import rosbag


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--schedule-source-time", action="store_true",
                        help="Reschedule IMU bag records by their original source stamps")
    args = parser.parse_args()
    source, output = Path(args.source), Path(args.output)
    if source.resolve() == output.resolve() or output.exists():
        raise RuntimeError("Refusing to overwrite source or an existing fixture")
    output.parent.mkdir(parents=True, exist_ok=True)
    before = sha256(source)
    intervals = [(6.0, 7.0), (12.0, 12.15)]
    counts, dropped = {}, [0, 0]
    boundaries = [{"source_stamp_before_sec": None, "source_stamp_after_sec": None}
                  for _ in intervals]
    with rosbag.Bag(str(source)) as original, rosbag.Bag(str(output), "w") as fault:
        begin = original.get_start_time()
        records = []
        first_source_stamp = None
        for topic, raw, stamp in original.read_messages(raw=True):
            elapsed = stamp.to_sec() - begin
            if elapsed >= args.duration:
                break
            source_stamp = None
            if topic == "/livox/imu":
                message = raw[-1]()
                message.deserialize(raw[1] if isinstance(raw[1], bytes) else raw[1][0])
                source_stamp = message.header.stamp.to_sec()
                if first_source_stamp is None:
                    first_source_stamp = source_stamp
                if args.schedule_source_time:
                    elapsed = source_stamp - first_source_stamp
                    if elapsed >= args.duration:
                        continue
                    import rospy
                    stamp = rospy.Time.from_sec(begin + elapsed)
            records.append((topic, raw, stamp, elapsed, source_stamp))
        records.sort(key=lambda row: row[2].to_nsec())
        for topic, raw, stamp, elapsed, source_stamp in records:
            if topic == "/livox/imu":
                interval = next((i for i, (start, end) in enumerate(intervals)
                                 if start <= elapsed < end), None)
                if interval is not None:
                    dropped[interval] += 1
                    continue
                for i, (start, end) in enumerate(intervals):
                    if elapsed < start:
                        boundaries[i]["source_stamp_before_sec"] = source_stamp
                    elif elapsed >= end and boundaries[i]["source_stamp_after_sec"] is None:
                        boundaries[i]["source_stamp_after_sec"] = source_stamp
            fault.write(topic, raw, stamp, raw=True)
            counts[topic] = counts.get(topic, 0) + 1
    after = sha256(source)
    if before != after:
        raise RuntimeError("Original source hash changed")
    manifest = {
        "source": str(source), "source_sha256_before": before,
        "source_sha256_after": after, "source_unchanged": True,
        "bag": str(output), "bag_sha256": sha256(output),
        "bag_start_time_sec": begin, "requested_duration_sec": args.duration,
        "written_messages": counts,
        "faults": [{"id": "long_1s" if i == 0 else "short_150ms",
                    "topic": "/livox/imu", "interval_sec": list(interval),
                    "removed_messages": dropped[i], **boundaries[i]}
                   for i, interval in enumerate(intervals)],
        "mutation": "Remove IMU records only; preserve lidar and source header stamps",
        "imu_bag_record_schedule": "source_stamp_relative_to_first" if args.schedule_source_time else "original_record_times",
    }
    output.with_suffix(".manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
