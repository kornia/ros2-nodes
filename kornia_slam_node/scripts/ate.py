"""Absolute trajectory error of kornia_slam's recorded odometry against EuRoC
ground truth (Umeyama alignment, SE3 or Sim3).

    python3 scripts/ate.py <recorded bag dir> <state_groundtruth_estimate0/data.csv> [--csv out.csv]
"""
import argparse

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

TOPIC = "/kornia_slam/odometry"


def read_odometry(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri), rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, Odometry)
        p = msg.pose.pose.position
        rows.append([msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, p.x, p.y, p.z])
    return np.array(rows, dtype=float)


def ate(est, gt, sim3):
    idx = np.searchsorted(gt[:, 0], est[:, 0]).clip(1, len(gt) - 1)
    idx = np.where(np.abs(gt[idx - 1, 0] - est[:, 0]) < np.abs(gt[idx, 0] - est[:, 0]), idx - 1, idx)
    ok = np.abs(gt[idx, 0] - est[:, 0]) < 2e7
    P, G = est[ok, 1:4], gt[idx[ok], 1:4]
    mp, mg = P.mean(0), G.mean(0)
    X, Y = P - mp, G - mg
    U, S, Vt = np.linalg.svd(Y.T @ X / len(P))
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = (S * np.diag(D)).sum() / X.var(0).sum() if sim3 else 1.0
    aligned = (s * (R @ X.T)).T + mg
    rmse = np.sqrt((np.linalg.norm(aligned - G, axis=1) ** 2).mean())
    return rmse, s, est[ok, 0], aligned, G


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("groundtruth")
    parser.add_argument("--csv", help="write stamp, aligned (SE3) and ground-truth positions")
    args = parser.parse_args()
    est = read_odometry(args.bag)
    gt = np.loadtxt(args.groundtruth, delimiter=",", comments="#", ndmin=2)
    path = np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum()
    print(f"poses={len(est)}  ground-truth path={path:.1f} m")
    for name, sim3 in (("SE3", False), ("Sim3", True)):
        rmse, s, *_ = ate(est, gt, sim3)
        print(f"{name:>4} ATE rmse={rmse:.3f} m  scale={s:.3f}")
    if args.csv:
        _, _, t, aligned, G = ate(est, gt, False)
        np.savetxt(args.csv, np.column_stack([t, aligned, G]), delimiter=",",
                   header="stamp_ns,x,y,z,gt_x,gt_y,gt_z", fmt=["%d"] + ["%.6f"] * 6)


main()
