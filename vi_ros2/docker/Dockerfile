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

# Toolchain 1.82: cargo-ament-build 0.1.11 ships a v4 Cargo.lock that Cargo
# < 1.78 cannot parse. 1.82 understands lockfile v4 and still builds rclrs and
# vi_node (rust-version = 1.75 is only a lower bound).
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
      sh -s -- -y --default-toolchain 1.82.0
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
