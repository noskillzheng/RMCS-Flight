#include <chrono>
#include <exception>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <hikcamera/image_capturer.hpp>

namespace {
/**
 * @class FPSCounter
 * @brief 帧率计数器 - 每秒统计一次FPS
 */
class FPSCounter {
public:
    FPSCounter()
        : frame_count_(0)
        , fps_(0.0)
        , last_time_(std::chrono::steady_clock::now()) {}

    /**
     * @brief 计数一帧，若满1秒则更新FPS
     * @return true 表示已过1秒，FPS已更新
     */
    bool count() {
        ++frame_count_;  // 帧数累加
        auto now      = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_);
        if (duration.count() >= 1) {  // 每满1秒更新一次
            fps_        = static_cast<double>(frame_count_) / duration.count();
            frame_count_ = 0;         // 重置计数
            last_time_   = now;       // 更新时间戳
            return true;
        }
        return false;
    }

    [[nodiscard]] double fps() const { return fps_; }

private:
    int frame_count_;     ///< 当前秒内的帧数
    double fps_;          ///< 计算得到的FPS
    std::chrono::steady_clock::time_point last_time_;  ///< 上次统计时间
};
} // namespace

int main() {
    std::cout << "=== Stage1: HikRobot Camera Smoke Test ===" << std::endl;
    try {
        // 1. 配置相机参数
        hikcamera::ImageCapturer::CameraProfile profile;
        profile.exposure_time = std::chrono::microseconds(2000);  // 曝光时间 2ms
        profile.gain          = 8.0F;                             // 增益
        profile.invert_image  = false;                            // 不翻转图像

        // 2. 创建相机捕获器（无同步模式）
        auto capturer = std::make_unique<hikcamera::ImageCapturer>(
            profile, nullptr, hikcamera::SyncMode::NONE);
        capturer->set_frame_rate_inner_trigger_mode(60);  // 设置60FPS

        // 3. 获取相机分辨率
        const auto [width, height] = capturer->get_width_height();
        std::cout << "[Camera] " << width << "x" << height << std::endl;

        // 4. 创建OpenCV显示窗口
        const char* window_name = "Stage1 - Camera Feed";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, 1280, 720);

        FPSCounter fps_counter;
        int frame_index = 0;  // 保存图像时的序号

        // 5. 主循环：采集图像并显示
        while (true) {
            auto image = capturer->read();  // 从相机读取一帧
            if (image.empty()) {
                std::cerr << "[Warn] empty frame, skip\n";
                continue;
            }

            // 克隆图像用于显示（避免修改原始图像）
            cv::Mat display = image.clone();

            // 每秒打印一次FPS
            if (fps_counter.count()) {
                std::cout << "[FPS] " << fps_counter.fps() << std::endl;
            }

            // 在图像上绘制FPS文字
            std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps_counter.fps()));
            cv::putText(
                display, fps_text, {10, 30},        // 位置：左上角偏移(10,30)
                cv::FONT_HERSHEY_SIMPLEX, 1.0,      // 字体和大小
                {0, 255, 0}, 2);                    // 绿色，粗细2

            // 显示图像
            cv::imshow(window_name, display);

            // 处理键盘输入
            const int key = cv::waitKey(1);  // 等待1ms
            if (key == 'q' || key == 27) {   // 'q'键或ESC键退出
                break;
            }
            if (key == 's') {  // 's'键保存当前帧
                const std::string filename = "capture_" + std::to_string(frame_index++) + ".jpg";
                cv::imwrite(filename, image);
                std::cout << "[Save] " << filename << std::endl;
            }
        }
        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}