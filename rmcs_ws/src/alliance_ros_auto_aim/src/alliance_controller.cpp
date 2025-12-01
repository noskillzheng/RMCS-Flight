#include "alliance_controller.hpp"
#include "alliance_initializer.hpp"
#include <Eigen/Core>
#include <opencv2/highgui.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "sync_data_processor.hpp"

namespace alliance_ros_auto_aim {

AllianceController::AllianceController(const rclcpp::NodeOptions& options)
    : rmcs_executor::Component()
    , rclcpp::Node("alliance_controller", options) {
    RCLCPP_INFO(get_logger(), "[DIAG] AllianceController constructor started");

    register_input("/camera/mat_stamped", mat_input_);
    register_input("/sync/camera_gimbal", sync_input_);

    register_output(
        "/gimbal/auto_aim/control_direction",
        control_direction_output_,
        Eigen::Vector3d::Zero());
    register_output(
        "/gimbal/auto_aim/fire_control",
        fire_control_output_,
        false);

    RCLCPP_INFO(get_logger(), "[DIAG] AllianceController initialized. "
        "Waiting for camera to be ready before starting SystemV1...");
}

AllianceController::~AllianceController() {
    if (AllianceInitializer::debug_mode()) {
        cv::destroyWindow(debug_window_name_);
    }
}

void AllianceController::update() {
    if (!mat_input_.ready() || !sync_input_.ready()) {
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), LOG_THROTTLE_PERIOD.count(),
            "AllianceController waiting for /camera/mat_stamped + /sync/camera_gimbal");
        return;
    }

    const auto& mat_stamped = *mat_input_;

    if (mat_stamped.mat.empty()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), LOG_THROTTLE_PERIOD.count(),
            "AllianceController: received empty image");
        return;
    }

    // Lazy initialization: create SystemV1 only after camera data is ready
    if (!adapter_) {
        RCLCPP_INFO(get_logger(),
            "[DIAG] Camera data ready. Initializing alliance_auto_aim SystemV1...");
        adapter_ = std::make_unique<AllianceAdapter>(get_logger());
        RCLCPP_INFO(get_logger(),
            "[DIAG] SystemV1 initialized successfully. Auto-aim system is now active.");
    }

    const auto sync_data = world_exe::ros::sync_data_process(*sync_input_);

    adapter_->publishImage(mat_stamped);
    adapter_->publishSyncData(sync_data);

    // One-shot consume semantics: cache is cleared after each call
    auto fire_control_opt = adapter_->getLatestFireControl();
    if (fire_control_opt.has_value()) {
        const auto& fire_cmd = fire_control_opt.value();

        *fire_control_output_ = fire_cmd.fire_allowance;

        // Only update gimbal direction when fire is allowed and direction is valid
        if (fire_cmd.fire_allowance && fire_cmd.gimbal_dir.norm() > 1e-6) {
            *control_direction_output_ = fire_cmd.gimbal_dir.normalized();
        }
        // Otherwise keep last valid direction to avoid NaN from normalizing zero vector
    } else {
        // No new fire control (target lost or system not publishing): deny fire
        // Safety default: do not continue using stale ALLOW state
        *fire_control_output_ = false;
    }

    if (AllianceInitializer::debug_mode()) {
        cv::imshow(debug_window_name_, mat_stamped.mat);
        cv::waitKey(1);

        if (fire_control_opt.has_value()) {
            const auto& fire_cmd = fire_control_opt.value();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[DEBUG] Fire: %s | Direction: (%.3f, %.3f, %.3f)",
                fire_cmd.fire_allowance ? "ALLOW" : "DENY",
                fire_cmd.gimbal_dir.x(), fire_cmd.gimbal_dir.y(), fire_cmd.gimbal_dir.z());
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[DEBUG] No target detected");
        }
    }
}

} // namespace alliance_ros_auto_aim

PLUGINLIB_EXPORT_CLASS(alliance_ros_auto_aim::AllianceController, rmcs_executor::Component)
