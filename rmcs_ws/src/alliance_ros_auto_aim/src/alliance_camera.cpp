#include "alliance_camera.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>


namespace alliance_ros_auto_aim {
AllianceHikCamera::AllianceHikCamera(const rclcpp::NodeOptions& options)
    : rmcs_executor::Component()
    , rclcpp::Node("alliance_hik_camera", options) {
    RCLCPP_INFO(get_logger(), "[DIAG] AllianceHikCamera constructor started");

    register_input("/tf", tf_input_);
    register_output("/camera/mat_stamped", mat_output_);
    register_output("/sync/camera_gimbal", sync_output_,
        world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas{});

    profile_.invert_image  = declare_parameter("invert_image", false);
    profile_.gain          = declare_parameter("gain", 16.9807);
    profile_.exposure_time = std::chrono::milliseconds(
        declare_parameter("exposure_time", 2000));

    capturer_ = std::make_unique<hikcamera::ImageCapturer>(
        profile_, nullptr, hikcamera::SyncMode::NONE);


    capturer_->set_frame_rate_inner_trigger_mode(100);
    RCLCPP_INFO(get_logger(), "[DIAG] AllianceHikCamera constructor finished");
}
void AllianceHikCamera::update() {
    if (!tf_input_.ready()) return;

    const auto stamp = now();
    const auto frame = capturer_->read();
    if (frame.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), LOG_THROTTLE_PERIOD.count(),
            "AllianceHikCamera: frame is empty");
        return;
    }

    auto& mat_stamped = *mat_output_;
    mat_stamped.Load(frame, world_exe::data::TimeStamp(
        std::chrono::nanoseconds(stamp.nanoseconds())));

    *sync_output_ = buildSyncPacket(*tf_input_, stamp);
}

world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas
AllianceHikCamera::buildSyncPacket(const rmcs_description::Tf& tf,
    const rclcpp::Time& stamp) const {
    world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas sync {};
    sync.time_stamp_in_nanosecond = stamp.nanoseconds();

    const auto cam_in_gimbal = fast_tf::cast<rmcs_description::GimbalCenterLink>(
        rmcs_description::CameraLink::Position{}, tf);
    const auto cam_rot_in_gimbal = fast_tf::cast<rmcs_description::GimbalCenterLink>(
        rmcs_description::CameraLink::Rotation{}, tf);

    sync.camera_to_gimbal_translation_x = cam_in_gimbal->x();
    sync.camera_to_gimbal_translation_y = cam_in_gimbal->y();
    sync.camera_to_gimbal_translation_z = cam_in_gimbal->z();
    sync.camera_to_gimbal_rotation_w    = cam_rot_in_gimbal->w();
    sync.camera_to_gimbal_rotation_x    = cam_rot_in_gimbal->x();
    sync.camera_to_gimbal_rotation_y    = cam_rot_in_gimbal->y();
    sync.camera_to_gimbal_rotation_z    = cam_rot_in_gimbal->z();

    const auto muzzle_in_gimbal = fast_tf::cast<rmcs_description::GimbalCenterLink>(
        rmcs_description::MuzzleLink::Position{}, tf);
    sync.gimbal_to_muzzle_translation_x = muzzle_in_gimbal->x();
    sync.gimbal_to_muzzle_translation_y = muzzle_in_gimbal->y();
    sync.gimbal_to_muzzle_translation_z = muzzle_in_gimbal->z();

    const auto muzzle_rot = fast_tf::cast<rmcs_description::GimbalCenterLink>(
        rmcs_description::MuzzleLink::Rotation{}, tf);
    sync.gimbal_to_muzzle_rotation_w = muzzle_rot->w();
    sync.gimbal_to_muzzle_rotation_x = muzzle_rot->x();
    sync.gimbal_to_muzzle_rotation_y = muzzle_rot->y();
    sync.gimbal_to_muzzle_rotation_z = muzzle_rot->z();

    return sync;
}
}

PLUGINLIB_EXPORT_CLASS(alliance_ros_auto_aim::AllianceHikCamera, rmcs_executor::Component)
