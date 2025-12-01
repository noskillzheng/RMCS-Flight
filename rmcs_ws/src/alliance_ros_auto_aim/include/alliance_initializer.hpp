#pragma once

#include <rmcs_executor/component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

namespace alliance_ros_auto_aim {

class AllianceInitializer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit AllianceInitializer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void update() override;

    static bool debug_mode() { return debug_mode_.load(std::memory_order_relaxed); }
    static bool target_is_blue() { return target_is_blue_.load(std::memory_order_relaxed); }

private:
    void declareParameters();
    void configureSystemV1Parameters();

    double fx_{}, fy_{}, cx_{}, cy_{};
    double k1_{}, k2_{}, k3_{};
    double bullet_speed_{}, gravity_{}, control_delay_{};
    std::string model_path_;
    std::string device_;

    static std::atomic<bool> debug_mode_;
    static std::atomic<bool> target_is_blue_;
};
}