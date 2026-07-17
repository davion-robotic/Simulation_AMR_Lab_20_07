#!/usr/bin/env python3

import os
import time
import math
import roslibpy

# ==========================
# CONFIG
# ==========================
HOST = "localhost"
PORT = 9090

# ==========================
# GLOBAL DATA
# ==========================
map_info = "Waiting..."
scan_info = "Waiting..."
tf_info = "Waiting..."
odom_info = "Waiting..."


# ==========================
# CALLBACKS
# ==========================

def on_map(msg):
    global map_info

    info = msg.get("info", {})

    width = info.get("width", 0)
    height = info.get("height", 0)
    resolution = info.get("resolution", 0.0)

    map_info = f"{width}x{height}  Res={resolution:.3f}"


def on_scan(msg):
    global scan_info

    ranges = msg.get("ranges", [])

    # Lọc None, NaN, Inf
    valid_ranges = [
        r for r in ranges
        if r is not None and isinstance(r, (int, float)) and math.isfinite(r)
    ]

    if not valid_ranges:
        scan_info = "No Valid Data"
        return

    scan_info = (
        f"N={len(valid_ranges)} "
        f"Min={min(valid_ranges):.2f} "
        f"Max={max(valid_ranges):.2f}"
    )


def on_tf(msg):
    global tf_info

    transforms = msg.get("transforms", [])

    if not transforms:
        tf_info = "No TF"
        return

    try:
        t = transforms[0]

        parent = t["header"]["frame_id"]
        child = t["child_frame_id"]

        pos = t["transform"]["translation"]

        tf_info = (
            f"{parent}->{child} "
            f"({pos['x']:.2f},{pos['y']:.2f})"
        )
    except Exception:
        tf_info = "Invalid TF"


def on_odom(msg):
    global odom_info

    try:
        pos = msg["pose"]["pose"]["position"]
        twist = msg["twist"]["twist"]

        odom_info = (
            f"({pos['x']:.2f},{pos['y']:.2f}) "
            f"V={twist['linear']['x']:.2f} "
            f"W={twist['angular']['z']:.2f}"
        )
    except Exception:
        odom_info = "Invalid Odom"


# ==========================
# DISPLAY
# ==========================

def print_dashboard():

    os.system("clear")

    print("=" * 120)
    print(
        f"{'MAP':<28}"
        f"{'SCAN':<32}"
        f"{'TF':<36}"
        f"{'ODOM':<24}"
    )
    print("=" * 120)

    print(
        f"{map_info:<28}"
        f"{scan_info:<32}"
        f"{tf_info:<36}"
        f"{odom_info:<24}"
    )

    print("=" * 120)


# ==========================
# MAIN
# ==========================

def main():

    client = roslibpy.Ros(host=HOST, port=PORT)

    client.run()

    print(f"Connected to ws://{HOST}:{PORT}")

    map_topic = roslibpy.Topic(
        client,
        "/map",
        "nav_msgs/OccupancyGrid"
    )

    scan_topic = roslibpy.Topic(
        client,
        "/scan",
        "sensor_msgs/LaserScan"
    )

    tf_topic = roslibpy.Topic(
        client,
        "/tf",
        "tf2_msgs/TFMessage"
    )

    odom_topic = roslibpy.Topic(
        client,
        "/odom",
        "nav_msgs/Odometry"
    )

    map_topic.subscribe(on_map)
    scan_topic.subscribe(on_scan)
    tf_topic.subscribe(on_tf)
    odom_topic.subscribe(on_odom)

    try:
        while True:
            print_dashboard()
            time.sleep(0.5)

    except KeyboardInterrupt:

        print("\nDisconnect...")

        map_topic.unsubscribe()
        scan_topic.unsubscribe()
        tf_topic.unsubscribe()
        odom_topic.unsubscribe()

        client.terminate()


if __name__ == "__main__":
    main()