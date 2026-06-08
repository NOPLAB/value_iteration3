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
# Pin ros2_rust to the commit our vi_node/src/main.rs is written against
# (ros2-rust/ros2_rust @ 2c6b926, the rclrs API we target). The sub-repos in
# ros2_rust_humble.repos still track their own branches, which is upstream's
# design and remains API-compatible with this commit.
RUN git clone https://github.com/ros2-rust/ros2_rust.git && \
    git -C ros2_rust checkout 2c6b92671a65a426aec9c3468c069a551329ca1b && \
    vcs import < ros2_rust/ros2_rust_humble.repos && \
    # ROOT CAUSE: the ros:humble-ros-base-jammy base image now ships Rust
    # bindings (share/<pkg>/rust/, marked `[package.metadata.ros-env] include`)
    # for the common interface packages (action_msgs, builtin_interfaces,
    # rcl_interfaces, rosgraph_msgs, unique_identifier_msgs, ...). ros2_rust's
    # .repos still imports those same interface repos to rebuild them. The
    # `ros-env` crate's build.rs enumerates every <pkg>/rust on AMENT_PREFIX_PATH,
    # so each duplicated package emits `pub mod <pkg>` twice -> E0428
    # "<pkg> defined multiple times" in the generated interfaces.rs and rclrs
    # fails to compile. Drop the workspace copies of the interface repos and let
    # ros-env source those bindings from /opt/ros/humble (which it does anyway).
    rm -rf ros2

WORKDIR /ros2_rust_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --merge-install --packages-up-to rclrs

WORKDIR /workspace
CMD ["/bin/bash"]
