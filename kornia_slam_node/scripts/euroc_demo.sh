#!/bin/bash
# Replays a EuRoC bag (scripts/euroc_to_bag.py) through image_proc into
# kornia_slam, records its odometry and reports the trajectory error.
#
#   scripts/euroc_demo.sh <bag> <groundtruth csv> <system config> <out dir> [rate]
#
# Needs a sourced ROS 2 and a release build (cargo build --release).
set -eu
BAG=$1 GT=$2 SYSTEM=$3 OUT=$4 RATE=${5:-0.5}
NODE=${NODE:-target/release/kornia_slam_node}
mkdir -p "$OUT"
rm -rf "$OUT/odometry"

pids=()
cleanup() { kill -TERM "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT

for cam in cam0 cam1; do
  ros2 run image_proc rectify_node --ros-args -r __node:=rectify_$cam \
    -r image:=/$cam/image_raw -r camera_info:=/$cam/camera_info -r image_rect:=/$cam/image_rect \
    > "$OUT/rectify_$cam.log" 2>&1 &
  pids+=($!)
done

params=(-p system_config:="$SYSTEM")
[ -f "$BAG.params.yaml" ] && params+=(--params-file "$BAG.params.yaml")
"$NODE" --ros-args "${params[@]}" \
  -r left/image_rect:=/cam0/image_rect -r left/camera_info:=/cam0/camera_info \
  -r right/image_rect:=/cam1/image_rect -r right/camera_info:=/cam1/camera_info \
  -r imu:=/imu0 \
  > "$OUT/kornia_slam.log" 2>&1 &
pids+=($!)

ros2 bag record -s mcap -o "$OUT/odometry" /kornia_slam/odometry > "$OUT/record.log" 2>&1 &
pids+=($!)

sleep 3
ros2 bag play "$BAG" --rate "$RATE" > "$OUT/play.log" 2>&1
sleep 3
cleanup
trap - EXIT

grep -h "dropped" "$OUT/kornia_slam.log" | tail -1 || true
python3 "$(dirname "$0")/ate.py" "$OUT/odometry" "$GT" --csv "$OUT/trajectory.csv" | tee "$OUT/ate.txt"
