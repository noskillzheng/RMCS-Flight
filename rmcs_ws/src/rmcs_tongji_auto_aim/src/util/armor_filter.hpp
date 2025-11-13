/**
 * @file src/util/armor_filter.hpp
 * @brief Tongji 识别结果的白名单过滤工具
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "interfaces/armor_in_image.hpp"
#include "data/armor_image_spaceing.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"

namespace rmcs_tongji_auto_aim::util {

/**
 * @brief 白名单过滤后的结果结构体
 */
struct ArmorFilterResult {
    std::shared_ptr<world_exe::interfaces::IArmorInImage> armors;
    world_exe::enumeration::CarIDFlag car_id_flag = world_exe::enumeration::CarIDFlag::None;
};

/**
 * @brief 按 RMCS 白名单语义过滤 Tongji 装甲板输出
 *
 * @param source    Tongji 识别器输出的装甲板集合
 * @param whitelist RMCS 白名单掩码
 * @return ArmorFilterResult 过滤后的装甲板集合及 CarIDFlag
 */
ArmorFilterResult filterArmorsByWhitelist(
    const std::shared_ptr<world_exe::interfaces::IArmorInImage>& source, uint8_t whitelist);

}  // namespace rmcs_tongji_auto_aim::util
