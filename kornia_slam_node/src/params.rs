//! Node parameters, set with `--ros-args -p name:=value` or a parameter file.

use std::path::PathBuf;
use std::sync::Arc;

use kornia_3d::pose::Pose3d;
use kornia_slam::{LoadError, SystemConfig};
use rclrs::{DeclarationError, Node};

use crate::convert::pose_from_xyz_quat;

pub struct Params {
    /// kornia-slam system configuration (RON); empty runs the defaults.
    pub system_config: Option<PathBuf>,
    /// Pose of the rectified left camera in the IMU frame; required with the IMU.
    pub camera_to_imu: Option<Pose3d>,
    /// Frames waiting for the tracker; the oldest is dropped beyond this.
    pub queue: usize,
    /// Seconds to wait for `CameraInfo` at startup.
    pub wait_s: f64,
    pub world_frame: String,
    pub camera_frame: String,
    pub publish_tf: bool,
    pub debug: bool,
}

#[derive(Debug, thiserror::Error)]
pub enum ParamError {
    #[error("parameter `{0}`: {1}")]
    Declaration(&'static str, DeclarationError),
    #[error("parameter `{0}`: {1}")]
    Invalid(&'static str, String),
    #[error("parameter `system_config`: {0}")]
    System(#[from] LoadError),
}

impl Params {
    pub fn declare(node: &Node) -> Result<Self, ParamError> {
        let string = |name: &'static str, default: &str| -> Result<String, ParamError> {
            node.declare_parameter::<Arc<str>>(name)
                .default(default.into())
                .read_only()
                .map(|p| p.get().to_string())
                .map_err(|e| ParamError::Declaration(name, e))
        };
        let flag = |name: &'static str, default: bool| -> Result<bool, ParamError> {
            node.declare_parameter(name)
                .default(default)
                .read_only()
                .map(|p| p.get())
                .map_err(|e| ParamError::Declaration(name, e))
        };

        let system_config = string("system_config", "")?;
        let queue = node
            .declare_parameter::<i64>("queue")
            .default(4)
            .read_only()
            .map_err(|e| ParamError::Declaration("queue", e))?
            .get();
        let wait_s = node
            .declare_parameter::<f64>("wait_s")
            .default(10.0)
            .read_only()
            .map_err(|e| ParamError::Declaration("wait_s", e))?
            .get();
        let camera_to_imu = node
            .declare_parameter::<Arc<[f64]>>("camera_to_imu")
            .optional()
            .map_err(|e| ParamError::Declaration("camera_to_imu", e))?
            .get()
            .map(|values| pose_from_xyz_quat(&values))
            .transpose()
            .map_err(|e| ParamError::Invalid("camera_to_imu", e))?;

        if queue < 1 {
            return Err(ParamError::Invalid("queue", format!("{queue} < 1")));
        }
        if !(wait_s.is_finite() && wait_s > 0.0) {
            return Err(ParamError::Invalid("wait_s", format!("{wait_s}")));
        }
        Ok(Self {
            system_config: (!system_config.is_empty()).then(|| system_config.into()),
            camera_to_imu,
            queue: queue as usize,
            wait_s,
            world_frame: string("world_frame", "map")?,
            camera_frame: string("camera_frame", "kornia_slam_camera")?,
            publish_tf: flag("publish_tf", true)?,
            debug: flag("debug", false)?,
        })
    }

    /// The system configuration, checked against the IMU extrinsic.
    pub fn system(&self) -> Result<SystemConfig, ParamError> {
        let config = match &self.system_config {
            Some(path) => SystemConfig::from_ron_file(path)?,
            None => SystemConfig::default(),
        };
        if config.sensors.imu && self.camera_to_imu.is_none() {
            return Err(ParamError::Invalid(
                "camera_to_imu",
                "required when the system uses the IMU".into(),
            ));
        }
        Ok(config)
    }
}
