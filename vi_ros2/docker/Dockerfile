FROM ros:humble-ros-base-jammy

# ros-humble-nav2-common は後段の nav2_msgs ソースビルドの buildtool 依存
# (nav2_package() CMake マクロ)。git/build-essential/colcon/vcstool/cmake は
# base イメージ同梱 (dpkg -s で確認済み) なのでここでは入れない。
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl pkg-config libclang-dev \
      python3-pip \
      libssl-dev \
      ros-humble-nav2-common \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir colcon-cargo colcon-ros-cargo

# Use the latest stable toolchain (not a pin): recent cargo-ament-build pulls
# deps (indexmap, ...) that need edition2024 (Rust >= 1.85) and a v4 Cargo.lock
# (Cargo >= 1.78). rclrs and vi_planner are edition 2021, so newer stable is fine;
# rust-version = 1.75 in vi_planner is only a lower bound.
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
      sh -s -- -y
ENV PATH=/root/.cargo/bin:$PATH

RUN cargo install --locked cargo-ament-build

WORKDIR /ros2_rust_ws/src
# Pin ros2_rust to the commit our vi_planner/src/main.rs is written against
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

# nav2_msgs (ComputePathToPose / FollowPath。vi_planner が使う) の Rust
# バインディング。apt バイナリの nav2_msgs は share/nav2_msgs/rust を含まない
# ため、ソースをこの ros2_rust ワークスペースでビルドして rosidl_generator_rs
# を通す。
RUN git clone --depth 1 --branch humble \
      https://github.com/ros-navigation/navigation2.git /tmp/navigation2 && \
    cp -a /tmp/navigation2/nav2_msgs /ros2_rust_ws/src/nav2_msgs && \
    rm -rf /tmp/navigation2
RUN . /opt/ros/humble/setup.sh && . /ros2_rust_ws/install/local_setup.sh && \
    colcon build --merge-install --packages-select nav2_msgs

WORKDIR /workspace
CMD ["/bin/bash"]
