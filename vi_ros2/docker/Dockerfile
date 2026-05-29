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

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
      sh -s -- -y --default-toolchain 1.75.0
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
