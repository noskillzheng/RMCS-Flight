#include "rmcs_core/mock_tf_provider.hpp"

#include <cmath>

namespace {
rclcpp::NodeOptions auto_declare_parameters(rclcpp::NodeOptions options) {
    options.automatically_declare_parameters_from_overrides(true);
    return options;
}
} // namespace

namespace rmcs_core::hardware {

MockTfProvider::MockTfProvider()
    : rmcs_executor::Component()
    , rclcpp::Node("mock_tf_provider", auto_declare_parameters(rclcpp::NodeOptions{})) {
    using namespace rmcs_description;

    register_output("/tf", tf_);

    const auto base_to_gimbal_offset =
        declare_parameter<std::vector<double>>("base_to_gimbal_offset", {0.0, 0.0, 0.0});
    const auto camera_offset =
        declare_parameter<std::vector<double>>("camera_offset", {0.16, 0.0, 0.15});
    const auto muzzle_offset =
        declare_parameter<std::vector<double>>("muzzle_offset", {0.0, 0.0, 0.18});

    const double initial_yaw_deg   = declare_parameter<double>("initial_yaw_deg", 0.0);
    const double initial_pitch_deg = declare_parameter<double>("initial_pitch_deg", 0.0);

    const double yaw   = initial_yaw_deg * M_PI / 180.0;
    const double pitch = initial_pitch_deg * M_PI / 180.0;

    tf_->set_transform<BaseLink, GimbalCenterLink>(Eigen::Translation3d{
        base_to_gimbal_offset.at(0), base_to_gimbal_offset.at(1), base_to_gimbal_offset.at(2)});

    tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{
        camera_offset.at(0), camera_offset.at(1), camera_offset.at(2)});
    tf_->set_transform<PitchLink, MuzzleLink>(Eigen::Translation3d{
        muzzle_offset.at(0), muzzle_offset.at(1), muzzle_offset.at(2)});

    tf_->set_state<GimbalCenterLink, YawLink>(yaw);
    tf_->set_state<YawLink, PitchLink>(pitch);

    RCLCPP_INFO(
        get_logger(),
        "MockTfProvider ready. base_to_gimbal=(%.3f, %.3f, %.3f), camera=(%.3f, %.3f, %.3f), "
        "muzzle=(%.3f, %.3f, %.3f), yaw=%.1f deg, pitch=%.1f deg",
        base_to_gimbal_offset.at(0), base_to_gimbal_offset.at(1), base_to_gimbal_offset.at(2),
        camera_offset.at(0), camera_offset.at(1), camera_offset.at(2), muzzle_offset.at(0),
        muzzle_offset.at(1), muzzle_offset.at(2), initial_yaw_deg, initial_pitch_deg);
}

void MockTfProvider::update() {
    // 静态 TF 提供者：当前不需要在 update 中修改任何状态。
}

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MockTfProvider, rmcs_executor::Component)
