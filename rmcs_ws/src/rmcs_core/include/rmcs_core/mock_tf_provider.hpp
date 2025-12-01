#pragma once

#include <rmcs_executor/component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_core::hardware {

/**
 * @brief MockTfProvider - 提供静态 /tf 以便在无底盘硬件时测试相机与自瞄链路。
 *
 * 只根据参数构造 Base -> GimbalCenter -> Pitch -> Camera/Muzzle 的几何关系，
 * 不依赖任何串口或下位机，适合作为实验环境中的 TF 桩件。
 */
class MockTfProvider
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MockTfProvider();
    ~MockTfProvider() override = default;

    void update() override;

private:
    rmcs_executor::Component::OutputInterface<rmcs_description::Tf> tf_;
};

} // namespace rmcs_core::hardware

