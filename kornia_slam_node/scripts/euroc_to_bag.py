"""Writes a EuRoC MAV sequence (ASL folder) as a ROS 2 bag, the way a stereo
camera driver would publish it: raw images, calibrated CameraInfo and the IMU.

    python3 scripts/euroc_to_bag.py <sequence dir> <output bag dir>

Topics: /cam0/image_raw, /cam0/camera_info, /cam1/image_raw, /cam1/camera_info
(mono8, plumb_bob with the stereo rectification R and P, as written by
`camera_calibration`) and /imu0. Also writes <output>.params.yaml with the
rectified left camera's pose in the IMU frame for kornia_slam's
`camera_to_imu`.
"""
import argparse
import csv
import os

import cv2
import numpy as np
import rosbag2_py
import yaml
from builtin_interfaces.msg import Time
from kornia_rs.image import Image as KorniaImage
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu


def load_camera(path):
    calib = yaml.safe_load(open(path))
    fu, fv, cu, cv = calib["intrinsics"]
    K = np.array([[fu, 0, cu], [0, fv, cv], [0, 0, 1.0]])
    D = np.array(calib["distortion_coefficients"] + [0.0])
    T_BS = np.array(calib["T_BS"]["data"]).reshape(4, 4)
    return K, D, T_BS, tuple(calib["resolution"])


def stamp(ns):
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def stamps(path):
    return [int(row[0]) for row in csv.reader(open(path)) if not row[0].startswith("#")]


def quaternion_xyzw(R):
    w = np.sqrt(max(0.0, 1.0 + np.trace(R))) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [x, y, z, w]


def camera_info(frame_id, size, K, D, R, P):
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = size
    info.distortion_model = "plumb_bob"
    info.d = [float(v) for v in D]
    info.k = [float(v) for v in K.flatten()]
    info.r = [float(v) for v in R.flatten()]
    info.p = [float(v) for v in P.flatten()]
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence")
    parser.add_argument("output")
    args = parser.parse_args()
    mav = os.path.join(args.sequence, "mav0")

    K0, D0, T0, size = load_camera(f"{mav}/cam0/sensor.yaml")
    K1, D1, T1, _ = load_camera(f"{mav}/cam1/sensor.yaml")
    T_10 = np.linalg.inv(T1) @ T0
    R0, R1, P0, P1, *_ = cv2.stereoRectify(
        K0, D0, K1, D1, size, T_10[:3, :3], T_10[:3, 3], flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )
    infos = {
        "cam0": camera_info("cam0", size, K0, D0, R0, P0),
        "cam1": camera_info("cam1", size, K1, D1, R1, P1),
    }

    # The rectified left camera in the IMU (body) frame: X_B = R_BS R0^T X_rect + t_BS.
    camera_to_imu = [*T0[:3, 3], *quaternion_xyzw(T0[:3, :3] @ R0.T)]
    params = {"kornia_slam": {"ros__parameters": {"camera_to_imu": [float(v) for v in camera_to_imu]}}}
    with open(args.output.rstrip("/") + ".params.yaml", "w") as f:
        yaml.safe_dump(params, f, default_flow_style=None)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.output, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    for name, kind in [
        ("/cam0/image_raw", "sensor_msgs/msg/Image"),
        ("/cam0/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/cam1/image_raw", "sensor_msgs/msg/Image"),
        ("/cam1/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/imu0", "sensor_msgs/msg/Imu"),
    ]:
        writer.create_topic(rosbag2_py.TopicMetadata(0, name, kind, "cdr"))

    frames = sorted(set(stamps(f"{mav}/cam0/data.csv")) & set(stamps(f"{mav}/cam1/data.csv")))
    imu = [
        (int(row[0]), [float(v) for v in row[1:]])
        for row in csv.reader(open(f"{mav}/imu0/data.csv"))
        if not row[0].startswith("#")
    ]
    events = [(t, 1, None) for t in frames] + [(t, 0, v) for t, v in imu]
    events.sort(key=lambda e: (e[0], e[1]))

    for t, kind, values in events:
        if kind == 0:
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = stamp(t), "imu0"
            w, a = msg.angular_velocity, msg.linear_acceleration
            w.x, w.y, w.z = values[:3]
            a.x, a.y, a.z = values[3:]
            writer.write("/imu0", serialize_message(msg), t)
            continue
        for cam in ("cam0", "cam1"):
            raw = KorniaImage.open(f"{mav}/{cam}/data/{t}.png", mode="L")
            img = Image()
            img.header.stamp, img.header.frame_id = stamp(t), cam
            img.height, img.width = raw.height, raw.width
            img.encoding, img.step, img.data = "mono8", img.width, raw.tobytes()
            writer.write(f"/{cam}/image_raw", serialize_message(img), t)
            info = infos[cam]
            info.header.stamp = stamp(t)
            writer.write(f"/{cam}/camera_info", serialize_message(info), t)
    print(f"{len(frames)} stereo frames, {len(imu)} IMU samples -> {args.output}")


main()
