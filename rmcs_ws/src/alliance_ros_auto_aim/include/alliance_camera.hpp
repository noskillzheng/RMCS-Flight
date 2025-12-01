#pragma once
#include <hikcamera/image_capturer.hpp>
#include <rmcs_executor/component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <data/mat_stamped.hpp>
#include <chrono>

#include "rmcs_description/tf_description.hpp"
#include "sync_data_processor.hpp"

namespace alliance_ros_auto_aim {

class AllianceHikCamera
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit AllianceHikCamera(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void update() override;

private:
    static constexpr std::chrono::milliseconds LOG_THROTTLE_PERIOD{2000};

    /**
     * @brief Build sync data packet containing camera-gimbal-muzzle transforms
     *
     * @details
     * Coordinate system definitions:
     * - RMCS TF tree uses ROS REP-103 standard: X-forward, Y-left, Z-up
     * - alliance_auto_aim expects same coordinate system: X-forward, Y-left, Z-up
     *
     * TF tree structure:
     * - CameraLink: Camera frame origin
     * - GimbalCenterLink: Gimbal center frame origin
     * - MuzzleLink: Muzzle frame origin
     *
     * Transform paths:
     * - camera_to_gimbal: Transform from camera frame to gimbal frame (translation+rotation)
     * - gimbal_to_muzzle: Transform from gimbal frame to muzzle frame (translation+rotation)
     *
     * These transforms are used for:
     * 1. Converting camera-detected armor positions to gimbal frame
     * 2. Computing ballistic compensation (distance and angle from muzzle to target)
     *
     * @param tf RMCS TF tree containing all frame transform relationships
     * @param stamp Timestamp for synchronizing TF transforms at image capture moment
     * @return Sync data packet with timestamp and two coordinate transforms
     */
    world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas
    buildSyncPacket(const rmcs_description::Tf& tf, const rclcpp::Time& stamp) const;

    rmcs_executor::Component::InputInterface<rmcs_description::Tf> tf_input_;
    rmcs_executor::Component::OutputInterface<world_exe::data::MatStamped> mat_output_;
    rmcs_executor::Component::OutputInterface<world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas> sync_output_;
    hikcamera::ImageCapturer::CameraProfile profile_;
    std::unique_ptr<hikcamera::ImageCapturer> capturer_;
};
}
