FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
      git curl build-essential pkg-config libclang-dev \
      python3-colcon-common-extensions \
      python3-colcon-mixin \
      python3-pip \
      python3-vcstool \
      libssl-dev cmake \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir colcon-cargo colcon-ros-cargo

# Use the latest stable toolchain (not a pin): recent cargo-ament-build pulls
# deps (indexmap, ...) that need edition2024 (Rust >= 1.85) and a v4 Cargo.lock
# (Cargo >= 1.78). rclrs and vi_node are edition 2021, so newer stable is fine;
# rust-version = 1.75 in vi_node is only a lower bound.
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
      sh -s -- -y
ENV PATH=/root/.cargo/bin:$PATH

RUN cargo install --locked cargo-ament-build

WORKDIR /ros2_rust_ws/src
RUN git clone https://github.com/ros2-rust/ros2_rust.git && \
    vcs import < ros2_rust/ros2_rust_humble.repos

WORKDIR /ros2_rust_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --packages-up-to rclrs

WORKDIR /workspace
CMD ["/bin/bash"]
