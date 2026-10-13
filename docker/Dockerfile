# ROS 2 Kilted with Rust, the message bindings rclrs needs, image_proc and
# rosbag2 (MCAP), plus the Python packages of scripts/.
FROM ros:kilted
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl libclang-dev pkg-config build-essential \
        ros-kilted-sensor-msgs ros-kilted-nav-msgs ros-kilted-geometry-msgs ros-kilted-tf2-msgs \
        ros-kilted-image-proc ros-kilted-rosbag2 ros-kilted-rosbag2-storage-mcap \
        python3-opencv python3-numpy python3-yaml python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir --break-system-packages kornia_rs
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain 1.89.0 --profile minimal -c clippy,rustfmt
ENV PATH=/root/.cargo/bin:$PATH
