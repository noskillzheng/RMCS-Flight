#include "alliance_initializer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "parameters/profile.hpp"
#include "parameters/params_system_v1.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {
rclcpp::NodeOptions auto_declare_parameters(rclcpp::NodeOptions options) {
    options.automatically_declare_parameters_from_overrides(true);
    return options;
}
} // namespace

namespace alliance_ros_auto_aim {

std::atomic<bool> AllianceInitializer::debug_mode_{false};
std::atomic<bool> AllianceInitializer::target_is_blue_{false};

AllianceInitializer::AllianceInitializer(const rclcpp::NodeOptions& options)
    : rmcs_executor::Component()
    , rclcpp::Node("alliance_initializer", auto_declare_parameters(options)) {

    declareParameters();

    configureSystemV1Parameters();

    RCLCPP_INFO(get_logger(), "[DIAG] AllianceInitializer constructor finished");
}


void AllianceInitializer::declareParameters() {
    bool debug_mode = get_parameter("debug_mode").as_bool();
    bool target_is_blue = get_parameter("target_is_blue").as_bool();
    model_path_ = get_parameter("model_path").as_string();
    device_ = get_parameter("device").as_string();

    fx_ = get_parameter("fx").as_double();
    fy_ = get_parameter("fy").as_double();
    cx_ = get_parameter("cx").as_double();
    cy_ = get_parameter("cy").as_double();
    k1_ = get_parameter("k1").as_double();
    k2_ = get_parameter("k2").as_double();
    k3_ = get_parameter("k3").as_double();

    bullet_speed_ = get_parameter("bullet_speed").as_double();
    gravity_ = get_parameter("gravity").as_double();
    control_delay_ = get_parameter("control_delay").as_double();

    debug_mode_.store(debug_mode, std::memory_order_relaxed);
    target_is_blue_.store(target_is_blue, std::memory_order_relaxed);
}

void AllianceInitializer::configureSystemV1Parameters() {
    world_exe::parameters::HikCameraProfile::set_width_height(1440, 1080);
    world_exe::parameters::HikCameraProfile::set_intrinsic_matrix(
        fx_, fy_, cx_, cy_, k1_, k2_, k3_);

    world_exe::parameters::ParamsForSystemV1::set_szu_model_path(model_path_);
    world_exe::parameters::ParamsForSystemV1::set_device(device_);
    world_exe::parameters::ParamsForSystemV1::set_velocity_begin(bullet_speed_);
    world_exe::parameters::ParamsForSystemV1::set_gravity(gravity_);
    world_exe::parameters::ParamsForSystemV1::set_control_delay_in_second(control_delay_);
    // Note: identifier convention is false=blue, true=red, so invert target_is_blue
    world_exe::parameters::ParamsForSystemV1::set_target_color(!target_is_blue_.load());

    // Note: SystemV1 is NOT built here!
    // It will be lazily initialized in AllianceAdapter constructor,
    // ensuring camera data is ready before starting the auto-aim system.

    RCLCPP_INFO(
        get_logger(), "AllianceInitializer: parameters configured. debug=%s target=%s. "
        "SystemV1 will be initialized when camera is ready.",
        debug_mode_.load() ? "true" : "false",
        target_is_blue_.load() ? "BLUE" : "RED");
}
void AllianceInitializer::update() {}
}

PLUGINLIB_EXPORT_CLASS(alliance_ros_auto_aim::AllianceInitializer, rmcs_executor::Component)
