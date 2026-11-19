fn main() {
    println!("cargo:rerun-if-env-changed=ROS_DISTRO");
    println!("cargo:rustc-check-cfg=cfg(ros_distro, values(\"jazzy\", \"lyrical\"))");
    match std::env::var("ROS_DISTRO").as_deref() {
        Ok("jazzy") => println!("cargo:rustc-cfg=ros_distro=\"jazzy\""),
        Ok("lyrical") => println!("cargo:rustc-cfg=ros_distro=\"lyrical\""),
        _ => {}
    }
}
