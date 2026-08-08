# RoboStack (pixi global env "ros") を bash で activate する。source して使う:
#   . scripts/ros2_native_env.sh
# Docker (/opt/ros/humble) の代替。rclrs ワークスペースは ~/ros2_rust_ws。
export CONDA_PREFIX="${CONDA_PREFIX:-$HOME/.pixi/envs/ros}"
export PATH="$CONDA_PREFIX/bin:$HOME/.cargo/bin:$PATH"
# activation スクリプト群は unbound 変数を参照するので nounset を外して source
_nounset=0
case "$-" in *u*) _nounset=1; set +u;; esac
for _f in "$CONDA_PREFIX"/etc/conda/activate.d/*.sh; do . "$_f"; done
# rclrs (ros2_rust) の colcon ビルド成果物
[ -f "$HOME/ros2_rust_ws/install/local_setup.sh" ] && \
    . "$HOME/ros2_rust_ws/install/local_setup.sh"
[ "$_nounset" = 1 ] && set -u
unset _f _nounset
# rclrs の bindgen が libclang を探す先 (conda の clangdev)
export LIBCLANG_PATH="$CONDA_PREFIX/lib"
