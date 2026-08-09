//! ROS メッセージ ⇄ vi_lib の型の変換。状態を持たない小物だけを置く
//! (どれも「同じ情報の詰め替え」で、判断はしない)。

use vi_lib::bridge::PoseView;
use vi_lib::msg::LaserScan as ViLaserScan;
use vi_lib::planner::PathPose;

use rclrs::*;

pub fn yaw_from_quat(q: &geometry_msgs::msg::Quaternion) -> f64 {
    let siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny_cosp.atan2(cosy_cosp)
}

pub fn pose_view_from(p: &geometry_msgs::msg::Pose) -> PoseView {
    PoseView { x: p.position.x, y: p.position.y, yaw_rad: yaw_from_quat(&p.orientation) }
}

/// `PathPose` 列 → `nav_msgs::msg::Path`。
pub fn poses_to_path(poses: &[PathPose], frame_id: &str, stamp: (i32, u32)) -> nav_msgs::msg::Path {
    let mut path = nav_msgs::msg::Path::default();
    path.header.frame_id = frame_id.into();
    path.header.stamp.sec = stamp.0;
    path.header.stamp.nanosec = stamp.1;
    path.poses = poses
        .iter()
        .map(|p| {
            let mut ps = geometry_msgs::msg::PoseStamped::default();
            ps.header.frame_id = frame_id.into();
            ps.header.stamp.sec = stamp.0;
            ps.header.stamp.nanosec = stamp.1;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.orientation.z = (p.yaw / 2.0).sin();
            ps.pose.orientation.w = (p.yaw / 2.0).cos();
            ps
        })
        .collect();
    path
}

/// 無効レンジ (inf / NaN / 非正) の差し替え値 [m]。ローカルウィンドウから
/// 十分遠く、セル座標化しても i32 に収まる値であればよい。
const INVALID_RANGE_M: f64 = 1.0e6;

/// sensor_msgs/LaserScan → vi_lib::LaserScan。ビーム角と添字の対応を
/// 保つため無効レンジは取り除かず [`INVALID_RANGE_M`] に差し替える
/// (`set_local_cost` がウィンドウ外として自然に無視する)。
pub fn vi_scan_from(msg: &sensor_msgs::msg::LaserScan) -> ViLaserScan {
    ViLaserScan {
        angle_min: msg.angle_min as f64,
        angle_increment: msg.angle_increment as f64,
        ranges: msg
            .ranges
            .iter()
            .map(|&r| {
                let r = r as f64;
                if r.is_finite() && r > 0.0 {
                    r
                } else {
                    INVALID_RANGE_M
                }
            })
            .collect(),
    }
}

pub fn stop_cmd(pub_cmd: &Publisher<geometry_msgs::msg::Twist>) {
    let _ = pub_cmd.publish(geometry_msgs::msg::Twist::default());
}

/// vi_lib の可視化描画済み OccupancyGrid → ROS メッセージ。
pub fn ros_grid_from(
    g: &vi_lib::msg::OccupancyGrid,
    frame_id: &str,
    stamp: (i32, u32),
) -> nav_msgs::msg::OccupancyGrid {
    let mut msg = nav_msgs::msg::OccupancyGrid::default();
    msg.header.frame_id = frame_id.into();
    msg.header.stamp.sec = stamp.0;
    msg.header.stamp.nanosec = stamp.1;
    msg.info.resolution = g.resolution as f32;
    msg.info.width = g.width as u32;
    msg.info.height = g.height as u32;
    msg.info.origin.position.x = g.origin_x;
    msg.info.origin.position.y = g.origin_y;
    msg.info.origin.orientation.x = g.origin_quat.x;
    msg.info.origin.orientation.y = g.origin_quat.y;
    msg.info.origin.orientation.z = g.origin_quat.z;
    msg.info.origin.orientation.w = g.origin_quat.w;
    msg.data = g.data.clone();
    msg
}
