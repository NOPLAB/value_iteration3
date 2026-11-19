ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# The ROS packages include the generated Rust bindings, including nav2_msgs.
RUN apt-get update && apt-get install -y --no-install-recommends \
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
      package_dir="${manifest%/rust/Cargo.toml}"; \
      touch "$index/${package_dir##*/}"; \
    done

WORKDIR /ros2_rust_ws
# Keep the existing rclrs API pin on Humble and Jazzy. Lyrical needs its own
# RCL bindings, but still ships messages generated for rosidl_runtime_rs 0.6.
RUN rev=2c6b92671a65a426aec9c3468c069a551329ca1b; \
    if [ "$ROS_DISTRO" = lyrical ]; then rev=2766eab6e1e9208a67e8570bfa5027ec38387534; fi; \
    git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust && \
    git -C src/ros2_rust checkout "$rev" && \
    if [ "$ROS_DISTRO" = lyrical ]; then \
      # ros-env 0.2.1 switched to rosidl_runtime_rs 0.7. Pin the last compatible
      # version in rclrs's manifest so both colcon and vi_planner use it.
      sed -i 's/^ros-env = "0\.2"$/ros-env = "=0.2.0"/' src/ros2_rust/rclrs/Cargo.toml; \
    fi && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build --merge-install --base-paths src/ros2_rust/rclrs --packages-select rclrs

WORKDIR /workspace
CMD ["/bin/bash"]
