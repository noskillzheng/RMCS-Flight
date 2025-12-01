#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <optional>
#include <opencv2/core/mat.hpp>

#include "core/event_bus.hpp"
#include "data/fire_control.hpp"
#include "data/mat_stamped.hpp"
#include "data/sync_data.hpp"

namespace alliance_ros_auto_aim {

/**
 * @brief Thread-safe adapter for Alliance Auto-Aim
 *
 * @details
 * This adapter class wraps alliance_auto_aim library's EventBus calls, providing:
 * 1. Thread safety: All EventBus operations are protected via mutex
 * 2. Unified error handling: Uses std::optional for potentially failing operations
 * 3. Logging adaptation: Converts internal events to RCLCPP logs
 * 4. Isolation layer: Prevents alliance_auto_aim implementation issues from propagating to RMCS
 *
 * @note
 * This class assumes alliance_auto_aim's SystemFactory-created system is single-threaded.
 * All EventBus calls should go through this adapter to ensure thread safety.
 *
 * @warning
 * alliance_auto_aim's EventBus has thread safety issues (concurrent std::vector access).
 * This adapter mitigates it by adding locks at the RMCS layer.
 */
class AllianceAdapter {
public:
    /**
     * @brief Constructor
     * @param logger RCLCPP logger for error reporting and debug info
     */
    explicit AllianceAdapter(rclcpp::Logger logger);

    /**
     * @brief Destructor - unsubscribes from all events
     */
    ~AllianceAdapter();

    // Non-copyable and non-movable
    AllianceAdapter(const AllianceAdapter&) = delete;
    AllianceAdapter& operator=(const AllianceAdapter&) = delete;
    AllianceAdapter(AllianceAdapter&&) = delete;
    AllianceAdapter& operator=(AllianceAdapter&&) = delete;

    /**
     * @brief Publish raw image to alliance_auto_aim's identification system
     *
     * @param mat_stamped Timestamped OpenCV Mat (BGR format)
     *
     * @note Thread-safe: internally protected by mutex
     */
    void publishImage(const world_exe::data::MatStamped& mat_stamped);

    /**
     * @brief Publish sync data (camera-gimbal-muzzle transforms) to alliance_auto_aim
     *
     * @param sync Sync data packet containing timestamp and coordinate transforms
     *
     * @note Thread-safe: internally protected by mutex
     */
    void publishSyncData(const world_exe::data::CameraGimbalMuzzleSyncData& sync);

    /**
     * @brief Get the latest fire control command
     *
     * @return std::optional<world_exe::data::FireControl>
     *         - If new fire control received, returns command data
     *         - If no command received or invalid, returns std::nullopt
     *
     * @note Thread-safe: internally protected by mutex
     * @note This method is polling-based and non-blocking
     * @note One-shot consume: value is cleared after retrieval
     */
    std::optional<world_exe::data::FireControl> getLatestFireControl();

private:
    /**
     * @brief Callback for FireControl events
     * @param cmd Fire control command
     *
     * @note Called when EventBus triggers (may be from different thread)
     */
    void onFireControlEvent(const world_exe::data::FireControl& cmd);

    rclcpp::Logger logger_;

    std::mutex mutex_;

    size_t fire_control_subscription_id_;

    std::optional<world_exe::data::FireControl> latest_fire_control_;
};

} // namespace alliance_ros_auto_aim
