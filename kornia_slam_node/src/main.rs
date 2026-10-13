//! ROS 2 node running [kornia-slam](https://github.com/kornia/kornia-slam)
//! on rectified mono or stereo images, optionally with an IMU, built on
//! [ros2_rust](https://github.com/ros2-rust/ros2_rust).
//!
//! ```text
//! ros2 run kornia_slam_node kornia_slam_node --ros-args \
//!     -p system_config:=config/stereo.ron -r left/image_rect:=/cam0/image_rect ...
//! ```
//!
//! The executor spins on its own thread; this thread runs the tracker on
//! frames as they become ready and publishes each tracked pose.

mod convert;
mod input;
mod output;
mod params;
mod sync;

use std::sync::mpsc;

use kornia_slam::{LoopClosureEvent, SensorFrame, SlamSystem, TrackingStatus};
use rclrs::{Context, CreateBasicExecutor, InitOptions, SpinOptions};

use input::Inbox;
use output::PosePublisher;
use params::Params;
use sync::seconds;

const NODE_NAME: &str = "kornia_slam";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::new(std::env::args(), InitOptions::default())?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;

    let params = Params::declare(&node)?;
    let config = params.system()?;
    let sensors = config.sensors;

    let (tx, rx) = mpsc::channel();
    let _subscriptions = input::subscribe(&node, sensors, &tx)?;
    drop(tx);
    let publisher = PosePublisher::new(&node, &params)?;
    std::thread::spawn(move || {
        for err in executor.spin(SpinOptions::default()) {
            eprintln!("[kornia_slam] executor: {err}");
        }
    });

    let mut inbox = Inbox::new(rx, sensors, params.queue);
    eprintln!(
        "[kornia_slam] {} waiting for CameraInfo",
        node.fully_qualified_name()
    );
    let mut rig = inbox.wait_for_rig(params.wait_s)?;
    if let (true, Some(camera_to_imu)) = (sensors.imu, &params.camera_to_imu) {
        rig = rig.with_imu(*camera_to_imu);
    }
    let mut system = SlamSystem::build(config, rig)?;
    system.set_debug(params.debug);
    eprintln!("[kornia_slam] tracking");

    let mut reported_drops = 0;
    for idx in 0.. {
        let (frame, imu_samples) = inbox.next_frame()?;
        let result = system.process(SensorFrame {
            idx,
            timestamp_sec: seconds(frame.stamp),
            image: &frame.image,
            right_image: frame.right_image.as_ref(),
            imu_samples: &imu_samples,
        })?;
        if result.status != TrackingStatus::Skipped {
            publisher.publish(frame.stamp, &result.pose_world_to_cam)?;
        }
        for line in system.drain_debug_messages() {
            eprintln!("{line}");
        }
        for event in system.drain_loop_closure_events() {
            log_loop_closure(&event);
        }
        let dropped = inbox.dropped();
        if dropped > reported_drops {
            eprintln!(
                "[kornia_slam] tracker behind: dropped {} frame(s), {dropped} in total",
                dropped - reported_drops
            );
            reported_drops = dropped;
        }
        if params.debug {
            eprintln!(
                "[{idx:>5}] {:?} pts={} {:.1}ms",
                result.status,
                system.num_active_map_points(),
                system.tracking_duration().as_secs_f64() * 1000.0
            );
        }
    }
    Ok(())
}

fn log_loop_closure(event: &LoopClosureEvent) {
    match event {
        LoopClosureEvent::Accepted { edge, applied } => eprintln!(
            "[kornia_slam] loop closure kf={} ~ kf={} inliers={} applied={applied}",
            edge.query_kf_idx, edge.candidate_kf_idx, edge.inliers,
        ),
        LoopClosureEvent::PgoFailed {
            query_kf_idx,
            candidate_kf_idx,
            reason,
        } => eprintln!(
            "[kornia_slam] pose graph failed kf={query_kf_idx} ~ kf={candidate_kf_idx}: {reason}"
        ),
    }
}
