#pragma once

#include <cstdint>
#include <Eigen/Geometry>

#include "data/sync_data.hpp"
#include "data/time_stamped.hpp"

namespace world_exe::ros {

/**
 * @struct SyncData_Feb_TimeCameraGimbal_8byteAlignas
 * @brief 8-byte aligned sync data packet for RMCS Component inter-communication
 * @details
 * - Contains timestamp (nanoseconds)
 * - Camera-to-gimbal transform (translation + rotation quaternion)
 * - Gimbal-to-muzzle transform (translation + rotation quaternion)
 */
struct alignas(8) SyncData_Feb_TimeCameraGimbal_8byteAlignas {
    int64_t time_stamp_in_nanosecond = 0;

    double camera_to_gimbal_translation_x = 0.0;
    double camera_to_gimbal_translation_y = 0.0;
    double camera_to_gimbal_translation_z = 0.0;
    double camera_to_gimbal_rotation_w = 1.0;
    double camera_to_gimbal_rotation_x = 0.0;
    double camera_to_gimbal_rotation_y = 0.0;
    double camera_to_gimbal_rotation_z = 0.0;

    double gimbal_to_muzzle_translation_x = 0.0;
    double gimbal_to_muzzle_translation_y = 0.0;
    double gimbal_to_muzzle_translation_z = 0.0;
    double gimbal_to_muzzle_rotation_w = 1.0;
    double gimbal_to_muzzle_rotation_x = 0.0;
    double gimbal_to_muzzle_rotation_y = 0.0;
    double gimbal_to_muzzle_rotation_z = 0.0;
};

/**
 * @brief Convert ROS sync data packet to alliance_auto_aim internal format
 * @param sync_packet 8-byte aligned sync data packet
 * @return CameraGimbalMuzzleSyncData internal sync data structure
 */
inline world_exe::data::CameraGimbalMuzzleSyncData sync_data_process(
    const SyncData_Feb_TimeCameraGimbal_8byteAlignas& sync_packet) {

    world_exe::data::CameraGimbalMuzzleSyncData result;

    result.camera_capture_begin_time_stamp =
        world_exe::data::TimeStamp::from_nanosec(sync_packet.time_stamp_in_nanosecond);

    Eigen::Quaterniond camera_to_gimbal_rotation(
        sync_packet.camera_to_gimbal_rotation_w,
        sync_packet.camera_to_gimbal_rotation_x,
        sync_packet.camera_to_gimbal_rotation_y,
        sync_packet.camera_to_gimbal_rotation_z);

    Eigen::Translation3d camera_to_gimbal_translation(
        sync_packet.camera_to_gimbal_translation_x,
        sync_packet.camera_to_gimbal_translation_y,
        sync_packet.camera_to_gimbal_translation_z);

    result.camera_to_gimbal = camera_to_gimbal_translation * camera_to_gimbal_rotation;

    Eigen::Quaterniond gimbal_to_muzzle_rotation(
        sync_packet.gimbal_to_muzzle_rotation_w,
        sync_packet.gimbal_to_muzzle_rotation_x,
        sync_packet.gimbal_to_muzzle_rotation_y,
        sync_packet.gimbal_to_muzzle_rotation_z);

    Eigen::Translation3d gimbal_to_muzzle_translation(
        sync_packet.gimbal_to_muzzle_translation_x,
        sync_packet.gimbal_to_muzzle_translation_y,
        sync_packet.gimbal_to_muzzle_translation_z);

    result.gimbal_to_muzzle = gimbal_to_muzzle_translation * gimbal_to_muzzle_rotation;

    return result;
}

}  // namespace world_exe::ros
