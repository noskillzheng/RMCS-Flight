/**
 * @file src/tongji_auto_aim_controller.cpp
 * @brief Main controller component for tongji auto-aim system
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node.hpp>
#include <Eigen/Dense>

#include <hikcamera/image_capturer.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_color.hpp>
#include <fast_tf/fast_tf.hpp>

// Tongji algorithm headers
#include "v1/identifier/identifier.hpp"
#include "tongji/solver/solver.hpp"
#include "tongji/predictor/car_predictor/car_predictor_manager.hpp"
#include "tongji/state_machine/state_machine.hpp"
#include "tongji/fire_controller/fire_controller.hpp"
#include "data/fire_control.hpp"
#include "data/predictor_update_package.hpp"
#include "data/sync_data.hpp"
#include "parameters/params_system_v1.hpp"
#include "parameters/profile.hpp"

// Local utilities
#include "util/yaml_generator.hpp"
#include "util/data_converter.hpp"
#include "util/fps_counter.hpp"

namespace rmcs_tongji_auto_aim {

using namespace world_exe;

/**
 * @brief 同济自瞄主控制器
 *
 * 采用双线程架构:
 * - 主线程: 从双缓冲读取结果,进行弹道计算,输出控制信号
 * - 视觉线程: 图像处理,装甲板识别,跟踪,火控计算
 */
class TongjiAutoAimController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    TongjiAutoAimController();
    ~TongjiAutoAimController() override;

    void update() override;

private:
    // === 视觉处理线程 ===
    void visionLoop();
    std::thread vision_thread_;
    std::atomic<bool> running_{true};

    // === 双缓冲机制 (lock-free) ===
    struct TargetFrame {
        data::FireControl control;
        std::chrono::steady_clock::time_point timestamp;
        bool valid{false};
    };
    TargetFrame target_buffer_[2];
    std::atomic<int> buffer_index_{0};

    // TF 双缓冲
    rmcs_description::Tf tf_buffer_[2];
    std::atomic<int> tf_index_{0};

    // === Input Interfaces ===
    InputInterface<size_t> update_count_;
    InputInterface<rmcs_msgs::RobotColor> target_color_;
    InputInterface<uint8_t> whitelist_;
    InputInterface<rmcs_description::Tf> tf_;

    // === Output Interfaces ===
    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> fire_control_;
    OutputInterface<double> debug_target_omega_;

    // === Tongji Algorithm Modules ===
    std::unique_ptr<hikcamera::ImageCapturer> capturer_;
    std::unique_ptr<v1::identifier::Identifier> identifier_;
    std::unique_ptr<tongji::solver::Solver> pnp_solver_;
    std::shared_ptr<tongji::predictor::CarPredictorManager> predictor_manager_;
    std::shared_ptr<tongji::state_machine::StateMachine> state_machine_;
    std::unique_ptr<tongji::fire_control::FireController> fire_controller_;

    // === Parameters ===
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, k3_;
    double yaw_error_, pitch_error_;
    double shoot_velocity_;
    double predict_sec_;

    // === Timing ===
    std::chrono::steady_clock::time_point last_timestamp_;
    util::FPSCounter fps_counter_;
};

// ==================== 构造函数实现 ====================

TongjiAutoAimController::TongjiAutoAimController()
    : rclcpp::Node(
          get_component_name(),
          rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

    RCLCPP_INFO(this->get_logger(), "TongjiAutoAimController initializing...");

    // === 注册 Input/Output 接口 ===
    register_input("/predefined/update_count", update_count_);
    register_input("/auto_aim/target_color", target_color_);
    register_input("/auto_aim/whitelist", whitelist_);
    register_input("/tf", tf_);

    register_output("/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
    register_output("/gimbal/auto_aim/fire_control", fire_control_, false);
    register_output("/debug/target_omega", debug_target_omega_, 0.0);

    // === 读取参数 ===
    // 相机内参
    fx_ = get_parameter("fx").as_double();
    fy_ = get_parameter("fy").as_double();
    cx_ = get_parameter("cx").as_double();
    cy_ = get_parameter("cy").as_double();
    k1_ = get_parameter("k1").as_double();
    k2_ = get_parameter("k2").as_double();
    k3_ = get_parameter("k3").as_double();

    // 相机配置
    auto exposure_time = get_parameter("exposure_time").as_int();
    auto invert_image = get_parameter("invert_image").as_bool();

    // 模型参数
    auto model_path = get_parameter("model_path").as_string();
    auto device = get_parameter("device").as_string();

    // 如果是相对路径,转换为绝对路径
    if (model_path[0] != '/') {
        model_path = ament_index_cpp::get_package_share_directory("rmcs_tongji_auto_aim")
                     + "/" + model_path;
    }

    // 弹道参数
    shoot_velocity_ = get_parameter("shoot_velocity").as_double();
    predict_sec_ = get_parameter("predict_sec").as_double();
    yaw_error_ = get_parameter("yaw_error").as_double();
    pitch_error_ = get_parameter("pitch_error").as_double();

    // Fire Controller 参数 (度数 → 弧度转换)
    auto control_delay = get_parameter("control_delay_s").as_double();
    auto bullet_speed = get_parameter("bullet_speed").as_double();
    auto yaw_offset_deg = get_parameter("yaw_offset_deg").as_double();
    auto pitch_offset_deg = get_parameter("pitch_offset_deg").as_double();
    auto comming_angle_deg = get_parameter("comming_angle_deg").as_double();
    auto leaving_angle_deg = get_parameter("leaving_angle_deg").as_double();
    auto auto_fire = get_parameter("auto_fire").as_bool();
    auto first_tolerance_deg = get_parameter("first_tolerance_deg").as_double();
    auto second_tolerance_deg = get_parameter("second_tolerance_deg").as_double();
    auto judge_distance = get_parameter("judge_distance").as_double();

    // === 初始化相机 ===
    hikcamera::ImageCapturer::CameraProfile profile;
    profile.invert_image = invert_image;
    profile.gain = 16.9807;
    profile.exposure_time = std::chrono::milliseconds(exposure_time);

    capturer_ = std::make_unique<hikcamera::ImageCapturer>(
        profile, nullptr, hikcamera::SyncMode::NONE);
    capturer_->set_frame_rate_inner_trigger_mode(100);

    // === 初始化 Tongji 全局参数 ===
    parameters::ParamsForSystemV1 params(fx_, fy_, cx_, cy_, k1_, k2_, k3_);
    parameters::ParamsForSystemV1::set_szu_model_path(model_path);
    parameters::ParamsForSystemV1::set_device(device);
    parameters::ParamsForSystemV1::set_velocity_begin(bullet_speed);
    parameters::ParamsForSystemV1::set_control_delay_in_second(control_delay);
    parameters::ParamsForSystemV1::set_gravity(9.7833);

    // 设置相机分辨率到 Profile (用于 PnP)
    auto [width, height] = capturer_->get_width_height();
    parameters::HikCameraProfile::set_width(width);
    parameters::HikCameraProfile::set_height(height);

    // === 生成 Fire Controller YAML 配置 ===
    std::string fire_control_yaml = util::YamlGenerator::generateFireControlYaml(
        control_delay,
        bullet_speed,
        util::DataConverter::deg2rad(yaw_offset_deg),
        util::DataConverter::deg2rad(pitch_offset_deg),
        util::DataConverter::deg2rad(comming_angle_deg),
        util::DataConverter::deg2rad(leaving_angle_deg),
        auto_fire,
        util::DataConverter::deg2rad(first_tolerance_deg),
        util::DataConverter::deg2rad(second_tolerance_deg),
        judge_distance);

    RCLCPP_INFO(this->get_logger(), "Generated fire control config: %s", fire_control_yaml.c_str());

    // === 初始化 Tongji 算法模块 ===
    identifier_ = std::make_unique<v1::identifier::Identifier>(
        model_path, device, width, height);

    pnp_solver_ = std::make_unique<tongji::solver::Solver>();

    predictor_manager_ = std::make_shared<tongji::predictor::CarPredictorManager>(
        "", 0.1);  // config_path 未使用

    state_machine_ = std::make_shared<tongji::state_machine::StateMachine>();

    fire_controller_ = std::make_unique<tongji::fire_control::FireController>(
        fire_control_yaml, state_machine_, predictor_manager_);

    last_timestamp_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "TongjiAutoAimController initialized successfully.");
}

// ==================== 析构函数实现 ====================

TongjiAutoAimController::~TongjiAutoAimController() {
    running_ = false;
    if (vision_thread_.joinable()) {
        vision_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "TongjiAutoAimController destroyed.");
}

// ==================== 主线程 update() ====================

void TongjiAutoAimController::update() {
    // 1. 主线程写入 TF 到双缓冲 (供视觉线程读取)
    tf_buffer_[!tf_index_.load()] = *tf_;
    tf_index_.store(!tf_index_.load());

    // 2. 首次更新时启动视觉处理线程
    if (*update_count_ == 0) {
        if (!target_color_.ready()) {
            RCLCPP_WARN(get_logger(), "target_color not ready!");
            throw std::runtime_error("target_color not ready");
        }

        vision_thread_ = std::thread([this]() { visionLoop(); });
        RCLCPP_INFO(get_logger(), "Vision thread started.");
    }

    // 3. 读取视觉线程的结果 (从双缓冲)
    int read_idx = buffer_index_.load();
    auto frame = target_buffer_[read_idx];

    if (!frame.valid) {
        *control_direction_ = Eigen::Vector3d::Zero();
        *fire_control_ = false;
        return;
    }

    // 4. 超时检测 (500ms)
    auto diff = std::chrono::steady_clock::now() - frame.timestamp;
    if (diff > std::chrono::milliseconds(500)) {
        *control_direction_ = Eigen::Vector3d::Zero();
        *fire_control_ = false;
        return;
    }

    // 5. 提取目标位置和方向
    // tongji FireControl 输出的是 gimbal_dir = [yaw, pitch, 0]
    double yaw = frame.control.gimbal_dir(0);
    double pitch = frame.control.gimbal_dir(1);

    // 转换为方向向量
    Eigen::Vector3d direction;
    direction.x() = cos(pitch) * cos(yaw);
    direction.y() = cos(pitch) * sin(yaw);
    direction.z() = sin(pitch);

    // 应用误差补偿
    auto yaw_axis = fast_tf::cast<rmcs_description::OdomImu>(
                        rmcs_description::PitchLink::DirectionVector(0, 0, 1), *tf_)
                        ->normalized();
    auto pitch_axis = fast_tf::cast<rmcs_description::OdomImu>(
                          rmcs_description::PitchLink::DirectionVector(0, 1, 0), *tf_)
                          ->normalized();

    auto delta_yaw = Eigen::AngleAxisd{yaw_error_, yaw_axis};
    auto delta_pitch = Eigen::AngleAxisd{pitch_error_, pitch_axis};
    direction = delta_pitch * (delta_yaw * direction);

    // 6. 输出控制信号
    *control_direction_ = direction.normalized();
    *fire_control_ = frame.control.fire_allowance;
}

// ==================== 视觉处理线程 ====================

void TongjiAutoAimController::visionLoop() {
    RCLCPP_INFO(get_logger(), "Vision loop started.");

    while (running_ && rclcpp::ok()) {
        try {
            // 1. 读取图像
            auto image = capturer_->read();
            auto timestamp = std::chrono::steady_clock::now();

            // 2. 读取 TF (从主线程的双缓冲)
            auto tf = tf_buffer_[tf_index_.load()];

            // 3. 装甲板识别
            auto [armors_in_image, car_id_flag] = identifier_->identify(image);

            if (car_id_flag == enumeration::ArmorIdFlag::None) {
                state_machine_->SetLostState();
                continue;
            }

            // 4. PnP 求解
            auto [camera_R, camera_t] = util::DataConverter::extractCameraToGimbal(tf);
            pnp_solver_->SetCamera2Gimbal(camera_R, camera_t);

            auto armors_in_camera = pnp_solver_->SolvePnp(armors_in_image);

            // 5. 构建同步数据包
            auto camera_to_gimbal = Eigen::Affine3d::Identity();
            camera_to_gimbal.linear() = camera_R;
            camera_to_gimbal.translation() = camera_t;

            data::CameraGimbalMuzzleSyncData sync_data;
            sync_data.camera_to_gimbal = camera_to_gimbal;
            sync_data.camera_capture_begin_time_stamp =
                data::TimeStamp(timestamp.time_since_epoch());

            auto update_pkg = std::make_shared<data::PredictorUpdatePackage>(
                sync_data, armors_in_camera);

            // 6. 预测器更新
            predictor_manager_->Update(update_pkg);

            // 7. 状态机更新
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                timestamp - last_timestamp_);
            state_machine_->Update(armors_in_image,
                                    enumeration::CarIDFlag::None,  // invincible_armors
                                    elapsed);

            // 8. 火控计算
            auto gimbal_yaw = util::DataConverter::extractGimbalYaw(tf);
            fire_controller_->UpdateGimbalPosition(gimbal_yaw);

            auto fire_control = fire_controller_->CalculateTarget(
                std::chrono::duration_cast<std::chrono::seconds>(elapsed));

            // 9. 原子写入双缓冲
            int write_idx = !buffer_index_.load();
            target_buffer_[write_idx].control = fire_control;
            target_buffer_[write_idx].timestamp = timestamp;
            target_buffer_[write_idx].valid = true;
            buffer_index_.store(write_idx);

            last_timestamp_ = timestamp;

            // FPS 统计
            if (fps_counter_.count()) {
                RCLCPP_DEBUG(get_logger(), "Vision FPS: %.1f", fps_counter_.getFPS());
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Vision loop error: %s", e.what());
        }
    }

    RCLCPP_INFO(get_logger(), "Vision loop exited.");
}

}  // namespace rmcs_tongji_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_tongji_auto_aim::TongjiAutoAimController, rmcs_executor::Component)
