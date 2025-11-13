/**
 * @file src/util/data_converter.hpp
 * @brief Data type converters between tongji and rmcs frameworks
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rmcs_description/tf_description.hpp>
#include <type_traits>

// 前向声明 tongji 类型
namespace world_exe::data {
struct FireControl;
}

namespace rmcs_tongji_auto_aim::util {

/**
 * @brief tongji 与 rmcs 数据类型转换工具
 */
class DataConverter {
public:
    /**
     * @brief 从 TF 树提取相机到云台的变换
     *
     * @param tf TF 树
     * @return std::pair<Eigen::Matrix3d, Eigen::Vector3d> 旋转矩阵和平移向量
     */
    static std::pair<Eigen::Matrix3d, Eigen::Vector3d>
    extractCameraToGimbal(const rmcs_description::Tf& tf) {
        // 从 camera_link 到云台基准系的变换
        // tongji 需要的是相机到云台的变换矩阵
        const auto camera_link_to_odom =
            fast_tf::lookup_transform<rmcs_description::CameraLink, rmcs_description::OdomImu>(tf);

        const auto yaw_link_to_odom =
            fast_tf::lookup_transform<rmcs_description::YawLink, rmcs_description::OdomImu>(tf);

        // Camera to Gimbal = Gimbal^-1 * Camera
        Eigen::Affine3d camera_to_gimbal =
            yaw_link_to_odom.inverse() * camera_link_to_odom;

        Eigen::Matrix3d R = camera_to_gimbal.rotation();
        Eigen::Vector3d t = camera_to_gimbal.translation();

        return {R, t};
    }

    /**
     * @brief 从 TF 树提取云台 Yaw 角
     *
     * @param tf TF 树
     * @return double Yaw 角 (弧度)
     */
    static double extractGimbalYaw(const rmcs_description::Tf& tf) {
        const auto yaw_transform =
            fast_tf::lookup_transform<rmcs_description::YawLink, rmcs_description::OdomImu>(tf);

        Eigen::Vector3d euler = toRotationMatrix(yaw_transform).eulerAngles(2, 1, 0);
        return euler(0);  // Yaw
    }

    /**
     * @brief 度数转弧度
     */
    static constexpr double deg2rad(double degrees) {
        return degrees * M_PI / 180.0;
    }

    /**
     * @brief 弧度转度数
     */
    static constexpr double rad2deg(double radians) {
        return radians * 180.0 / M_PI;
    }
private:
    template <typename>
    static constexpr bool kAlwaysFalse = false;

    template <typename TransformT>
    static Eigen::Matrix3d toRotationMatrix(const TransformT& transform) {
        using Transform = std::decay_t<TransformT>;
        if constexpr (
            std::is_same_v<Transform, Eigen::Isometry3d>
            || std::is_same_v<Transform, Eigen::Affine3d>) {
            return transform.rotation();
        } else if constexpr (
            std::is_same_v<Transform, Eigen::Quaterniond>
            || std::is_same_v<Transform, Eigen::AngleAxisd>) {
            return transform.toRotationMatrix();
        } else {
            static_assert(kAlwaysFalse<Transform>, "Unsupported transform type");
        }
    }
};

}  // namespace rmcs_tongji_auto_aim::util
