# ros2-nodes

ROS 2 nodes for [kornia](https://github.com/kornia), written in Rust on
[ros2_rust](https://github.com/ros2-rust/ros2_rust) (rclrs 0.8).

A cargo workspace; each node is its own crate and ROS package.

| Node | |
| --- | --- |
| [`kornia_slam_node`](kornia_slam_node) | visual / visual-inertial odometry with [kornia-slam](https://github.com/kornia/kornia-slam) |

## Build

Needs ROS 2 Kilted or newer, whose apt packages ship the Rust message crates, sourced:

```bash
source /opt/ros/kilted/setup.bash
cargo build --release
```

`docker/Dockerfile` is a `ros:kilted` image with Rust and everything the nodes and their scripts need.
