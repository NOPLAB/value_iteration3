ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Use the Ubuntu Azure mirror for CI runners. Leave ROS and Ubuntu Ports
# sources in place.
RUN set -e; \
    for source in /etc/apt/sources.list /etc/apt/sources.list.d/ubuntu.sources; do \
      if [ -f "$source" ]; then \
        sed -i -E 's@https?://(archive|security)\.ubuntu\.com/ubuntu@https://azure.archive.ubuntu.com/ubuntu@g' "$source"; \
      fi; \
    done; \
    apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update; \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      curl pkg-config libclang-dev libssl-dev python3-pip \
      "ros-${ROS_DISTRO}-nav2-msgs" \
    && rm -rf /var/lib/apt/lists/*

# Jazzy and Lyrical mark the system Python as externally managed. colcon uses
# that interpreter, so install its plugins into it in this disposable image.
RUN PIP_BREAK_SYSTEM_PACKAGES=1 python3 -m pip install --no-cache-dir \
      colcon-cargo colcon-ros-cargo

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
      sh -s -- -y
ENV PATH=/root/.cargo/bin:$PATH

RUN cargo install --locked cargo-ament-build

# colcon-ros-cargo needs the legacy index to generate local crates.io patches.
RUN prefix="/opt/ros/${ROS_DISTRO}"; \
    index="$prefix/share/ament_index/resource_index/rust_packages"; \
    mkdir -p "$index"; \
    for manifest in "$prefix"/share/*/rust/Cargo.toml; do \
      if [ "$ROS_DISTRO" = lyrical ]; then \
        # The packaged bindings still request 0.6. Lyrical's C primitive
        # sequences have extra fields, which rosidl_runtime_rs 0.7 mirrors.
        sed -i 's/^rosidl_runtime_rs = "0\.6"$/rosidl_runtime_rs = "0.7"/' "$manifest"; \
      fi; \
      package_dir="${manifest%/rust/Cargo.toml}"; \
      touch "$index/${package_dir##*/}"; \
    done

WORKDIR /ros2_rust_ws
# Keep the existing rclrs pin on Humble and Jazzy. Lyrical requires its own
# RCL bindings and rosidl_runtime_rs 0.7.
RUN rev=2c6b92671a65a426aec9c3468c069a551329ca1b; \
    if [ "$ROS_DISTRO" = lyrical ]; then rev=7a097bb349a193a96f620e3e94bb2224ffd698af; fi; \
    git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust && \
    git -C src/ros2_rust checkout "$rev" && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --merge-install --base-paths src/ros2_rust/rclrs --packages-select rclrs

WORKDIR /workspace
CMD ["/bin/bash"]
