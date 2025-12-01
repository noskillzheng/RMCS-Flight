#pragma once

#include <rmcs_executor/component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <data/mat_stamped.hpp>
#include <Eigen/Core>
#include <string>
#include <memory>
#include <chrono>

#include "sync_data_processor.hpp"
#include "alliance_adapter.hpp"

namespace alliance_ros_auto_aim {

/**
 * @brief alliance_controller - Bridge node connecting RMCS to alliance_auto_aim
 *
 * @details
 * - Subscribes to /camera/mat_stamped (MatStamped)
 * - Subscribes to /sync/camera_gimbal (SyncData_Feb_TimeCameraGimbal_8byteAlignas)
 * - Publishes to alliance_auto_aim internal system via AllianceAdapter
 * - Listens to FireControl events and outputs gimbal control commands
 *
 * @note Uses AllianceAdapter as isolation layer for thread safety and error handling
 */
class AllianceController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit AllianceController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AllianceController() override;
    void update() override;

private:
    static constexpr std::chrono::milliseconds LOG_THROTTLE_PERIOD{2000};

    rmcs_executor::Component::InputInterface<world_exe::data::MatStamped> mat_input_;
    rmcs_executor::Component::InputInterface<world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas> sync_input_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector3d> control_direction_output_;
    rmcs_executor::Component::OutputInterface<bool> fire_control_output_;

    const std::string debug_window_name_{"Alliance Auto-Aim Camera"};

    std::unique_ptr<AllianceAdapter> adapter_;
};

} // namespace alliance_ros_auto_aim
