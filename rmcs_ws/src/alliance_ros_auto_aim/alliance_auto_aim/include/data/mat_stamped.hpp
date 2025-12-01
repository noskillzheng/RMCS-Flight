#pragma once

#include "data/time_stamped.hpp"

#include <ctime>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

namespace world_exe::data {
struct MatStamped {
    data::TimeStamp stamp{};
    cv::Mat mat {};
    
    MatStamped()                                = default;
    MatStamped(const MatStamped&)               = delete;
    MatStamped& operator=(const MatStamped&)    = delete;
    
    MatStamped(cv::Mat&& image, data::TimeStamp ts)
    {
        mat = std::move(image);
        stamp = ts;
    }

    void Load(const cv::Mat& image, data::TimeStamp now)
    {
        // 防御性检查：如果 mat 已存在但类型不兼容，先释放避免 copyTo 断言失败
        if (!mat.empty() && mat.type() != image.type()) {
            mat.release();
        }
        image.copyTo(mat);
        stamp = now;
    }
};
}