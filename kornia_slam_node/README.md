# kornia_slam_node

Runs [kornia-slam](https://github.com/kornia/kornia-slam) on rectified mono or stereo images,
optionally with an IMU, and publishes the camera pose.

```bash
ros2 run kornia_slam_node kornia_slam_node --ros-args \
    -p system_config:=kornia_slam_node/config/stereo.ron \
    -r left/image_rect:=/cam0/image_rect -r left/camera_info:=/cam0/camera_info \
    -r right/image_rect:=/cam1/image_rect -r right/camera_info:=/cam1/camera_info
```

## Topics

| Topic | Type | |
| --- | --- | --- |
| `left/image_rect`, `left/camera_info` | `sensor_msgs/Image`, `CameraInfo` | rectified left camera |
| `right/image_rect`, `right/camera_info` | `sensor_msgs/Image`, `CameraInfo` | stereo only |
| `imu` | `sensor_msgs/Imu` | with `imu: true` |
| `~/odometry` (out) | `nav_msgs/Odometry` | camera pose, stamped with its image stamp |
| `/tf` (out) | `tf2_msgs/TFMessage` | `world_frame -> camera_frame` |

The camera and stereo baseline come from `CameraInfo.P`. Stereo views pair on identical stamps;
IMU samples are split per frame. Once `queue` frames wait, the oldest is dropped with a warning.

## Parameters

| Name | Default | |
| --- | --- | --- |
| `system_config` | defaults | kornia-slam `SystemConfig` (RON), see `config/` |
| `camera_to_imu` | | `[x, y, z, qx, qy, qz, qw]`, rectified left camera in the IMU frame; needed with the IMU |
| `queue` | 4 | frames waiting for the tracker |
| `wait_s` | 10.0 | seconds to wait for `CameraInfo` |
| `world_frame` / `camera_frame` | `map` / `kornia_slam_camera` | |
| `publish_tf` | true | |
| `debug` | false | |

## EuRoC demo

```bash
python3 kornia_slam_node/scripts/euroc_to_bag.py <V1_01_easy/mav0> data/V1_01_easy
kornia_slam_node/scripts/euroc_demo.sh data/V1_01_easy <mav0/state_groundtruth_estimate0/data.csv> \
    kornia_slam_node/config/stereo_imu.ron runs/v101 0.25
```

The bag replays raw images through `image_proc` rectify nodes into the node, records the odometry
and prints the trajectory error.
