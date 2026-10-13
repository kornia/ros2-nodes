//! Publishes the tracked camera pose as `nav_msgs/Odometry` on `~/odometry`
//! and as a TF transform from the world frame to the camera frame.
//!
//! The world frame is the SLAM map, which inertial initialization may rotate
//! and rescale. The camera frame is the (left) rectified camera's optical
//! frame: x right, y down, z forward. Link both into an existing TF tree with
//! static transforms.

use kornia_3d::pose::Pose3d;
use kornia_algebra::QuatF64;
use rclrs::{IntoPrimitiveOptions, Node, Publisher, RclrsError};
use ros_env::builtin_interfaces::msg::Time;
use ros_env::geometry_msgs::msg::{Point, Pose, Quaternion, Transform, TransformStamped, Vector3};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::std_msgs::msg::Header;
use ros_env::tf2_msgs::msg::TFMessage;

use crate::params::Params;
use crate::sync::StampNs;

pub struct PosePublisher {
    odometry: Publisher<Odometry>,
    tf: Option<Publisher<TFMessage>>,
    world_frame: String,
    camera_frame: String,
}

impl PosePublisher {
    pub fn new(node: &Node, params: &Params) -> Result<Self, RclrsError> {
        let tf = if params.publish_tf {
            Some(node.create_publisher("/tf".keep_last(100))?)
        } else {
            None
        };
        Ok(Self {
            odometry: node.create_publisher("~/odometry")?,
            tf,
            world_frame: params.world_frame.clone(),
            camera_frame: params.camera_frame.clone(),
        })
    }

    /// Publishes the camera pose of the frame captured at `stamp`.
    pub fn publish(&self, stamp: StampNs, pose_world_to_cam: &Pose3d) -> Result<(), RclrsError> {
        let header = Header {
            stamp: ros_time(stamp),
            frame_id: self.world_frame.clone(),
        };
        let (t, q) = camera_in_world(pose_world_to_cam);
        let rotation = Quaternion {
            x: q[0],
            y: q[1],
            z: q[2],
            w: q[3],
        };
        let odometry = Odometry {
            header: header.clone(),
            child_frame_id: self.camera_frame.clone(),
            pose: ros_env::geometry_msgs::msg::PoseWithCovariance {
                pose: Pose {
                    position: Point {
                        x: t[0],
                        y: t[1],
                        z: t[2],
                    },
                    orientation: rotation.clone(),
                },
                ..Default::default()
            },
            ..Default::default()
        };
        self.odometry.publish(odometry)?;
        let Some(tf) = &self.tf else {
            return Ok(());
        };
        tf.publish(TFMessage {
            transforms: vec![TransformStamped {
                header,
                child_frame_id: self.camera_frame.clone(),
                transform: Transform {
                    translation: Vector3 {
                        x: t[0],
                        y: t[1],
                        z: t[2],
                    },
                    rotation,
                },
            }],
        })
    }
}

fn ros_time(stamp: StampNs) -> Time {
    Time {
        sec: stamp.div_euclid(1_000_000_000) as i32,
        nanosec: stamp.rem_euclid(1_000_000_000) as u32,
    }
}

/// Camera position and orientation (`[x, y, z, w]`) in the world frame.
fn camera_in_world(pose_world_to_cam: &Pose3d) -> ([f64; 3], [f64; 4]) {
    let cam_to_world = pose_world_to_cam.inverse();
    let t = cam_to_world.translation;
    let q = QuatF64::from_mat3(&cam_to_world.rotation).normalize();
    ([t.x, t.y, t.z], q.to_array())
}

#[cfg(test)]
mod tests {
    use kornia_algebra::{Mat3F64, Vec3F64};

    use super::*;

    #[test]
    fn stamps_keep_the_nanosecond() {
        let time = ros_time(1_403_715_273_262_142_976);
        assert_eq!((time.sec, time.nanosec), (1_403_715_273, 262_142_976));
    }

    #[test]
    fn the_camera_pose_is_its_world_position_and_orientation() {
        // Camera at (1, 2, 3), turned 90° about the world z axis.
        let cam_to_world = Pose3d::from_rt(
            Mat3F64::from_cols(
                Vec3F64::new(0.0, 1.0, 0.0),
                Vec3F64::new(-1.0, 0.0, 0.0),
                Vec3F64::new(0.0, 0.0, 1.0),
            ),
            Vec3F64::new(1.0, 2.0, 3.0),
        );
        let (t, q) = camera_in_world(&cam_to_world.inverse());
        assert!(
            t.iter()
                .zip([1.0, 2.0, 3.0])
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
        let half = std::f64::consts::FRAC_1_SQRT_2;
        assert!(
            q.iter()
                .zip([0.0, 0.0, half, half])
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
    }
}
