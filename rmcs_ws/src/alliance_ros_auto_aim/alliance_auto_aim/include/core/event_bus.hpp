#pragma once

#include <algorithm>
#include <atomic>
#include <execution>
#include <functional>
#include <string>
#include <unordered_map>

namespace world_exe::core {

class EventBus {

public:
    enum class BusStatus : int { OK, Block = -1, NoSubscriptor = 1 };

    template <class TData>
    static world_exe::core::EventBus::BusStatus Publish(
        const std::string event_name, const TData& data) {
        return EventBusImpl<TData>::GetInstance().Publish(event_name, data);
    }

    template <typename TData>
    static size_t Subscript(const std::string& event_name, std::function<void(const TData&)>&& func) {
        return EventBusImpl<TData>::GetInstance().Subscript(event_name, std::move(func));
    }

    template <typename TData>
    static void Unsubscribe(const std::string& event_name, size_t subscription_id) {
        return EventBusImpl<TData>::GetInstance().Unsubscribe(event_name, subscription_id);
    }

private:
    template <typename TData> class EventBusImpl {
    private:
        EventBusImpl()                               = default;
        EventBusImpl(const EventBusImpl&)            = delete;
        EventBusImpl& operator=(const EventBusImpl&) = delete;

        std::unordered_map<std::string,
            std::tuple<std::atomic_bool, std::vector<std::function<void(const TData&)>>>>
            delegates = {};

    public:
        world_exe::core::EventBus::BusStatus Publish(
            const std::string& event_name, const TData& data) {
            if (!delegates.contains(event_name)) [[unlikely]] {
                return EventBus::BusStatus::NoSubscriptor;
            }
            auto& [flag, vec] = delegates[event_name];
            auto flag_val     = false;

            // if (!flag.compare_exchange_weak(flag_val, true))
            //     return EventBus::BusStatus::Block;
            // std::for_each(std::execution::par_unseq, vec.begin(), vec.end(),
            //     [&data](const auto& func) { func(data); });
            for (const auto& a : vec)
                a(data);
            // flag.store(false);
            return EventBus::BusStatus::OK;
        }
        size_t Subscript(const std::string event_name, std::function<void(const TData&)>&& func) {
            const auto& [iterator, create_falg] = delegates.try_emplace(event_name);
            auto& [flag, vec]                   = iterator->second;
            if (!create_falg) flag.store(false);
            vec.emplace_back(func);
            return vec.size() - 1;  // 返回订阅 ID
        }

        void Unsubscribe(const std::string& event_name, size_t subscription_id) {
            if (!delegates.contains(event_name)) return;
            auto& [flag, vec] = delegates[event_name];
            if (subscription_id < vec.size()) {
                vec.erase(vec.begin() + subscription_id);
            }
        }
        static EventBusImpl<TData>& GetInstance() {
            static EventBusImpl<TData> instance_;
            return instance_;
        }
    };
};
} // namespace world_exe::core