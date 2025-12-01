#include "alliance_adapter.hpp"
#include "alliance_initializer.hpp"
#include "parameters/params_system_v1.hpp"
#include "core/system_factory.hpp"
#include "enum/system_version.hpp"

namespace alliance_ros_auto_aim {

AllianceAdapter::AllianceAdapter(rclcpp::Logger logger)
    : logger_(logger)
    , fire_control_subscription_id_(0) {

    RCLCPP_INFO(logger_, "AllianceAdapter: building SystemV1...");

    const bool debug_mode = AllianceInitializer::debug_mode();
    world_exe::core::SystemFactory::Build(
        debug_mode ? world_exe::enumeration::SystemVersion::V1Debug
                   : world_exe::enumeration::SystemVersion::V1);

    RCLCPP_INFO(logger_, "AllianceAdapter: SystemV1 built successfully (debug=%s)",
                debug_mode ? "true" : "false");

    fire_control_subscription_id_ = world_exe::core::EventBus::Subscript<world_exe::data::FireControl>(
        world_exe::parameters::ParamsForSystemV1::fire_control_event,
        [this](const auto& fire_cmd) { onFireControlEvent(fire_cmd); });

    RCLCPP_INFO(logger_, "AllianceAdapter: subscribed to FireControl events (ID: %zu)",
                fire_control_subscription_id_);
}

AllianceAdapter::~AllianceAdapter() {
    world_exe::core::EventBus::Unsubscribe<world_exe::data::FireControl>(
        world_exe::parameters::ParamsForSystemV1::fire_control_event,
        fire_control_subscription_id_);

    RCLCPP_INFO(logger_, "AllianceAdapter: unsubscribed from FireControl events");
}

void AllianceAdapter::publishImage(const world_exe::data::MatStamped& mat_stamped) {
    if (mat_stamped.mat.empty()) {
        RCLCPP_WARN(logger_, "AllianceAdapter::publishImage: received empty image, skipping");
        return;
    }

    // Do NOT lock here! EventBus::Publish is synchronous and triggers SystemV1::solve(),
    // which publishes fire_control_event, invoking onFireControlEvent().
    // If we hold mutex_ here, onFireControlEvent() would try to acquire the same lock,
    // causing a deadlock (same thread recursive locking on std::mutex).

    auto status = world_exe::core::EventBus::Publish(
        world_exe::parameters::ParamsForSystemV1::raw_image_event,
        mat_stamped);

    switch (status) {
        case world_exe::core::EventBus::BusStatus::OK:
            break;
        case world_exe::core::EventBus::BusStatus::NoSubscriptor:
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                "AllianceAdapter::publishImage: no subscribers for raw_image_event");
            break;
        case world_exe::core::EventBus::BusStatus::Block:
            RCLCPP_ERROR(logger_, "AllianceAdapter::publishImage: EventBus blocked (concurrent access?)");
            break;
    }
}

void AllianceAdapter::publishSyncData(const world_exe::data::CameraGimbalMuzzleSyncData& sync) {
    // Do NOT lock here, same reason as publishImage()

    auto status = world_exe::core::EventBus::Publish(
        world_exe::parameters::ParamsForSystemV1::camera_capture_transforms,
        sync);

    switch (status) {
        case world_exe::core::EventBus::BusStatus::OK:
            break;
        case world_exe::core::EventBus::BusStatus::NoSubscriptor:
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                "AllianceAdapter::publishSyncData: no subscribers for camera_capture_transforms");
            break;
        case world_exe::core::EventBus::BusStatus::Block:
            RCLCPP_ERROR(logger_, "AllianceAdapter::publishSyncData: EventBus blocked");
            break;
    }
}

std::optional<world_exe::data::FireControl> AllianceAdapter::getLatestFireControl() {
    std::lock_guard<std::mutex> lock(mutex_);

    // One-shot consume semantics: return current value then clear cache.
    // This ensures that when solve() stops publishing FireControl (target lost),
    // the next call returns nullopt, allowing upper layer to show "No target detected".
    auto result = latest_fire_control_;
    latest_fire_control_.reset();
    return result;
}

void AllianceAdapter::onFireControlEvent(const world_exe::data::FireControl& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_fire_control_ = cmd;

    RCLCPP_DEBUG(logger_,
        "AllianceAdapter: received FireControl - fire_allowance=%s, gimbal_dir=[%.3f, %.3f, %.3f]",
        cmd.fire_allowance ? "true" : "false",
        cmd.gimbal_dir.x(), cmd.gimbal_dir.y(), cmd.gimbal_dir.z());
}

} // namespace alliance_ros_auto_aim
