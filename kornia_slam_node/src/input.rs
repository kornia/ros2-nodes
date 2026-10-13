//! Subscriptions to the rectified images, their `CameraInfo` and the IMU.
//!
//! Callbacks run on the executor thread: they convert each message and hand it
//! over a channel. [`Inbox`] assembles tracker frames on the SLAM thread.

use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::time::{Duration, Instant};

use kornia_algebra::Vec3F64;
use kornia_image::Image;
use kornia_sensors::SensorRig;
use kornia_sensors::imu::ImuMeasurement;
use kornia_slam::{CameraSelection, SensorSelection};
use rclrs::{IntoPrimitiveOptions, Node, RclrsError, Subscription};
use ros_env::sensor_msgs::msg::{CameraInfo, Image as ImageMsg, Imu};

use crate::convert::{camera_from_projection, gray_image, stereo_baseline_m};
use crate::sync::{Frame, FrameSync, ImuBuffer, StampNs, seconds, stamp_ns};

pub const LEFT_IMAGE: &str = "left/image_rect";
pub const LEFT_INFO: &str = "left/camera_info";
pub const RIGHT_IMAGE: &str = "right/image_rect";
pub const RIGHT_INFO: &str = "right/camera_info";
pub const IMU: &str = "imu";

/// How long a frame waits for the IMU sample that closes its interval.
const IMU_WAIT: Duration = Duration::from_millis(100);
const IMAGE_DEPTH: u32 = 10;
const IMU_DEPTH: u32 = 400;

pub enum Message {
    Left(StampNs, Image<u8, 1>),
    Right(StampNs, Image<u8, 1>),
    Imu(ImuMeasurement),
    LeftProjection([f64; 12]),
    RightProjection([f64; 12]),
    Invalid(String),
}

#[derive(Debug, thiserror::Error)]
pub enum InputError {
    #[error("no CameraInfo on `{0}` within {1} s")]
    NoCameraInfo(&'static str, f64),
    #[error("{0}")]
    Invalid(String),
    #[error("the ROS 2 executor stopped")]
    Disconnected,
}

#[derive(Clone, Copy)]
enum Side {
    Left,
    Right,
}

pub struct Subscriptions {
    _left: Subscription<ImageMsg>,
    _left_info: Subscription<CameraInfo>,
    _right: Option<(Subscription<ImageMsg>, Subscription<CameraInfo>)>,
    _imu: Option<Subscription<Imu>>,
}

pub fn subscribe(
    node: &Node,
    sensors: SensorSelection,
    tx: &Sender<Message>,
) -> Result<Subscriptions, RclrsError> {
    let right = if sensors.cameras == CameraSelection::Stereo {
        Some((
            image_subscription(node, RIGHT_IMAGE, Side::Right, tx.clone())?,
            info_subscription(node, RIGHT_INFO, Side::Right, tx.clone())?,
        ))
    } else {
        None
    };
    let imu = if sensors.imu {
        Some(imu_subscription(node, tx.clone())?)
    } else {
        None
    };
    Ok(Subscriptions {
        _left: image_subscription(node, LEFT_IMAGE, Side::Left, tx.clone())?,
        _left_info: info_subscription(node, LEFT_INFO, Side::Left, tx.clone())?,
        _right: right,
        _imu: imu,
    })
}

/// Messages from the callbacks, assembled into frames on the caller's thread.
pub struct Inbox {
    rx: Receiver<Message>,
    stereo: bool,
    frames: FrameSync,
    imu: Option<ImuBuffer>,
}

impl Inbox {
    pub fn new(rx: Receiver<Message>, sensors: SensorSelection, queue: usize) -> Self {
        let stereo = sensors.cameras == CameraSelection::Stereo;
        Self {
            rx,
            stereo,
            frames: FrameSync::new(stereo, queue),
            imu: sensors.imu.then(ImuBuffer::default),
        }
    }

    /// The rectified camera (and stereo baseline) from the first `CameraInfo`
    /// messages. Calibration updates after this are ignored.
    pub fn wait_for_rig(&mut self, wait_s: f64) -> Result<SensorRig, InputError> {
        let deadline = Instant::now() + Duration::from_secs_f64(wait_s);
        let (mut left, mut right) = (None, None);
        loop {
            match (left, right) {
                (Some(left), Some(right)) if self.stereo => {
                    return Ok(rectified_rig(&left)?.with_stereo_baseline(
                        stereo_baseline_m(&right).map_err(InputError::Invalid)?,
                    ));
                }
                (Some(left), _) if !self.stereo => return rectified_rig(&left),
                _ => {}
            }
            let timeout = deadline.saturating_duration_since(Instant::now());
            match self.rx.recv_timeout(timeout) {
                Ok(Message::LeftProjection(p)) => left = Some(p),
                Ok(Message::RightProjection(p)) => right = Some(p),
                Ok(message) => self.accept(message)?,
                Err(RecvTimeoutError::Timeout) => {
                    let missing = if left.is_none() {
                        LEFT_INFO
                    } else {
                        RIGHT_INFO
                    };
                    return Err(InputError::NoCameraInfo(missing, wait_s));
                }
                Err(RecvTimeoutError::Disconnected) => return Err(InputError::Disconnected),
            }
        }
    }

    /// Blocks until a frame is ready, with the IMU samples since the last one.
    pub fn next_frame(&mut self) -> Result<(Frame, Vec<ImuMeasurement>), InputError> {
        let frame = loop {
            while let Ok(message) = self.rx.try_recv() {
                self.accept(message)?;
            }
            if let Some(frame) = self.frames.pop() {
                break frame;
            }
            let message = self.rx.recv().map_err(|_| InputError::Disconnected)?;
            self.accept(message)?;
        };
        let imu = self.imu_until(seconds(frame.stamp))?;
        Ok((frame, imu))
    }

    /// Frames dropped because the tracker fell behind.
    pub fn dropped(&self) -> usize {
        self.frames.dropped()
    }

    fn accept(&mut self, message: Message) -> Result<(), InputError> {
        match message {
            Message::Left(stamp, image) => self.frames.push_left(stamp, image),
            Message::Right(stamp, image) => self.frames.push_right(stamp, image),
            Message::Imu(sample) => {
                if let Some(imu) = &mut self.imu {
                    imu.push(sample);
                }
            }
            Message::LeftProjection(_) | Message::RightProjection(_) => {}
            Message::Invalid(reason) => return Err(InputError::Invalid(reason)),
        }
        Ok(())
    }

    /// Waits briefly for the IMU sample that closes the frame's interval, so
    /// late IMU messages are not split across frames.
    fn imu_until(&mut self, timestamp_sec: f64) -> Result<Vec<ImuMeasurement>, InputError> {
        let deadline = Instant::now() + IMU_WAIT;
        while self
            .imu
            .as_ref()
            .is_some_and(|imu| !imu.covers(timestamp_sec))
        {
            let timeout = deadline.saturating_duration_since(Instant::now());
            match self.rx.recv_timeout(timeout) {
                Ok(message) => self.accept(message)?,
                Err(RecvTimeoutError::Timeout) => break,
                Err(RecvTimeoutError::Disconnected) => return Err(InputError::Disconnected),
            }
        }
        Ok(self
            .imu
            .as_mut()
            .map(|imu| imu.drain_until(timestamp_sec))
            .unwrap_or_default())
    }
}

fn image_subscription(
    node: &Node,
    topic: &str,
    side: Side,
    tx: Sender<Message>,
) -> Result<Subscription<ImageMsg>, RclrsError> {
    node.create_subscription(
        topic.best_effort().keep_last(IMAGE_DEPTH),
        move |msg: ImageMsg| {
            tx.send(image_message(&msg, side)).ok();
        },
    )
}

fn info_subscription(
    node: &Node,
    topic: &str,
    side: Side,
    tx: Sender<Message>,
) -> Result<Subscription<CameraInfo>, RclrsError> {
    node.create_subscription(topic.best_effort().keep_last(1), move |msg: CameraInfo| {
        let message = match side {
            Side::Left => Message::LeftProjection(msg.p),
            Side::Right => Message::RightProjection(msg.p),
        };
        tx.send(message).ok();
    })
}

fn imu_subscription(node: &Node, tx: Sender<Message>) -> Result<Subscription<Imu>, RclrsError> {
    node.create_subscription(IMU.best_effort().keep_last(IMU_DEPTH), move |msg: Imu| {
        let stamp = stamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
        let (w, a) = (msg.angular_velocity, msg.linear_acceleration);
        tx.send(Message::Imu(ImuMeasurement {
            timestamp: seconds(stamp),
            gyro: Vec3F64::new(w.x, w.y, w.z),
            accel: Vec3F64::new(a.x, a.y, a.z),
        }))
        .ok();
    })
}

fn rectified_rig(left: &[f64; 12]) -> Result<SensorRig, InputError> {
    Ok(SensorRig::new(
        camera_from_projection(left).map_err(InputError::Invalid)?,
    ))
}

fn image_message(msg: &ImageMsg, side: Side) -> Message {
    let stamp = stamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
    match gray_image(&msg.encoding, msg.width, msg.height, msg.step, &msg.data) {
        Ok(image) => match side {
            Side::Left => Message::Left(stamp, image),
            Side::Right => Message::Right(stamp, image),
        },
        Err(reason) => Message::Invalid(reason),
    }
}
