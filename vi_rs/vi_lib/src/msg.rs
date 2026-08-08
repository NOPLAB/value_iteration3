//! 本家が参照する ROS メッセージのフィールドのみを持つ最小代替型。

/// `geometry_msgs::Quaternion` 相当 (計算には使わず echo されるだけ)。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// `nav_msgs::OccupancyGrid` の使用フィールドのみ。
/// `data` は ROS の int8 (0=free、それ以外=占有/unknown)。
#[derive(Clone, Debug, Default)]
pub struct OccupancyGrid {
    pub width: i32,
    pub height: i32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub origin_quat: Quaternion,
    pub data: Vec<i8>,
}

/// `sensor_msgs::LaserScan` の使用フィールドのみ。
#[derive(Clone, Debug, Default)]
pub struct LaserScan {
    pub angle_min: f64,
    pub angle_increment: f64,
    pub ranges: Vec<f64>,
}
