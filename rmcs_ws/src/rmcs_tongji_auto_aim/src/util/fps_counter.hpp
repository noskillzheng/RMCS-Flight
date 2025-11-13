/**
 * @file src/util/fps_counter.hpp
 * @brief FPS counter utility
 */

#pragma once

#include <chrono>

namespace rmcs_tongji_auto_aim::util {

/**
 * @brief 简单的 FPS 计数器
 */
class FPSCounter {
public:
    FPSCounter()
        : frame_count_(0)
        , last_time_(std::chrono::steady_clock::now()) {}

    /**
     * @brief 记录一帧
     * @return true 如果已经过去 1 秒,可以读取 FPS
     */
    bool count() {
        frame_count_++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_time_).count();

        if (elapsed >= 1000) {
            fps_ = frame_count_ * 1000.0 / elapsed;
            frame_count_ = 0;
            last_time_ = now;
            return true;
        }
        return false;
    }

    /**
     * @brief 获取当前 FPS
     */
    double getFPS() const { return fps_; }

private:
    int frame_count_;
    double fps_{0.0};
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace rmcs_tongji_auto_aim::util
