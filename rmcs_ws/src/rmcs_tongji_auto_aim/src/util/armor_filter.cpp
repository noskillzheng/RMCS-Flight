/**
 * @file src/util/armor_filter.cpp
 */

#include "util/armor_filter.hpp"

#include <algorithm>
#include <bit>

namespace rmcs_tongji_auto_aim::util {

namespace {

using world_exe::data::ArmorImageSpacing;
using world_exe::enumeration::ArmorIdFlag;
using world_exe::enumeration::CarIDFlag;
using world_exe::interfaces::IArmorInImage;

constexpr std::array<ArmorIdFlag, 8> kArmorOrder = {
    ArmorIdFlag::Hero,
    ArmorIdFlag::Engineer,
    ArmorIdFlag::InfantryIII,
    ArmorIdFlag::InfantryIV,
    ArmorIdFlag::InfantryV,
    ArmorIdFlag::Sentry,
    ArmorIdFlag::Base,
    ArmorIdFlag::Outpost,
};

constexpr uint8_t kWhitelistHero        = 0x1;
constexpr uint8_t kWhitelistEngineer    = 0x2;
constexpr uint8_t kWhitelistInfantryIII = 0x4;
constexpr uint8_t kWhitelistInfantryIV  = 0x8;
constexpr uint8_t kWhitelistInfantryV   = 0x10;
constexpr uint8_t kWhitelistSentry      = 0x20;
constexpr uint8_t kWhitelistOutpost     = 0x40;
constexpr uint8_t kWhitelistBase        = 0x80;

bool isArmorAllowed(ArmorIdFlag flag, uint8_t whitelist) {
    switch (flag) {
    case ArmorIdFlag::Hero: return (whitelist & kWhitelistHero) == 0;
    case ArmorIdFlag::Engineer: return (whitelist & kWhitelistEngineer) == 0;
    case ArmorIdFlag::InfantryIII: return (whitelist & kWhitelistInfantryIII) == 0;
    case ArmorIdFlag::InfantryIV: return (whitelist & kWhitelistInfantryIV) == 0;
    case ArmorIdFlag::InfantryV: return (whitelist & kWhitelistInfantryV) == 0;
    case ArmorIdFlag::Sentry: return (whitelist & kWhitelistSentry) == 0;
    case ArmorIdFlag::Outpost: return (whitelist & kWhitelistOutpost) == 0;
    case ArmorIdFlag::Base: return (whitelist & kWhitelistBase) != 0;
    default: return true;
    }
}

class FilteredArmorInImage : public IArmorInImage {
public:
    FilteredArmorInImage(
        const std::shared_ptr<IArmorInImage>& source, uint8_t whitelist)
        : timestamp_(source->GetTimeStamp()) {
        for (size_t idx = 0; idx < kArmorOrder.size(); ++idx) {
            const auto flag = kArmorOrder[idx];
            if (!isArmorAllowed(flag, whitelist)) continue;
            const auto& armors = source->GetArmors(flag);
            if (armors.empty()) continue;
            filtered_[idx] = armors;
        }
    }

    const world_exe::data::TimeStamp& GetTimeStamp() const override { return timestamp_; }

    const std::vector<ArmorImageSpacing>& GetArmors(
        const ArmorIdFlag& armor_id) const override {
        for (size_t idx = 0; idx < kArmorOrder.size(); ++idx) {
            if (armor_id == kArmorOrder[idx]) {
                return filtered_[idx];
            }
        }
        return empty_;
    }

private:
    world_exe::data::TimeStamp timestamp_;
    std::array<std::vector<ArmorImageSpacing>, kArmorOrder.size()> filtered_{};
    std::vector<ArmorImageSpacing> empty_;
};

}  // namespace

ArmorFilterResult filterArmorsByWhitelist(
    const std::shared_ptr<IArmorInImage>& source, uint8_t whitelist) {
    ArmorFilterResult result;
    if (!source) {
        return result;
    }

    auto filtered = std::make_shared<FilteredArmorInImage>(source, whitelist);
    CarIDFlag flags = CarIDFlag::None;

    for (size_t idx = 0; idx < kArmorOrder.size(); ++idx) {
        if (!filtered->GetArmors(kArmorOrder[idx]).empty()) {
            flags = static_cast<CarIDFlag>(
                static_cast<uint32_t>(flags) | static_cast<uint32_t>(kArmorOrder[idx]));
        }
    }

    result.armors = std::move(filtered);
    result.car_id_flag = flags;
    return result;
}

}  // namespace rmcs_tongji_auto_aim::util
