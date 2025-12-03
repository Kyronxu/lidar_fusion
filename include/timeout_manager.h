#pragma once

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <functional>
#include <atomic>
#include <thread>
#include <ros/ros.h>

namespace SystemMonitor {

class TimeoutManager {
private:
    using Clock = std::chrono::steady_clock;

    struct MonitorInfo {
        Clock::time_point last_received;   // 最后接收时间
        Clock::time_point last_triggered;  // 上次触发回调的时间（控制间隔核心）
        std::chrono::milliseconds timeout; // 超时阈值
        std::function<void(size_t trigger_count)> callback;
        std::atomic<size_t> trigger_count{0};  // 已触发次数
        size_t max_trigger_times;             // 最大触发次数（0=无限次，默认1次）
        std::chrono::milliseconds start_delay; // 注册后监测启动延迟（ms）
        Clock::time_point register_time;      // 注册时间（仅用于计算延迟）

        MonitorInfo(Clock::time_point received,
                   std::chrono::milliseconds t,
                   std::function<void(size_t)> cb,
                   size_t max_trigger,
                   std::chrono::milliseconds delay)
            : last_received(received)
            , last_triggered(received - t)  // 初始化：避免首次触发时间差异常
            , timeout(t)
            , callback(std::move(cb))
            , max_trigger_times(max_trigger) 
            , start_delay(delay)
            , register_time(Clock::now()){}
    };

    std::unordered_map<std::string, MonitorInfo> monitors_;
    mutable std::mutex mutex_;
    boost::asio::io_context io_context_;
    boost::asio::steady_timer timer_;
    std::thread io_thread_;
    std::atomic<bool> is_running_{false};
    const std::chrono::milliseconds CHECK_INTERVAL{20};     // 轮询周期

public:
    explicit TimeoutManager() : timer_(io_context_) {
        is_running_ = true;

        io_thread_ = std::thread([this]() {
            ROS_INFO("TimeoutManager IO thread started");
            while (is_running_.load()) {
                try {
                    io_context_.run();
                    break;
                } catch (const std::exception& e) {
                    ROS_ERROR("TimeoutManager IO context error: %s", e.what());
                    io_context_.reset();
                }
            }
            ROS_INFO("TimeoutManager IO thread stopped");
        });

        startTimer();
        ROS_INFO("TimeoutManager initialized with check interval: %ldms", CHECK_INTERVAL.count());
    }

    /**
     * @brief 注册接口：带触发次数的回调
     * @param topic 话题名称
     * @param timeout 超时阈值（ms）
     * @param callback 回调函数
     * @param max_trigger_times 最大触发次数（默认1次，0=无限次）
     * @return 注册是否成功
     */
    bool RegisterTopic(const std::string& topic,
                       std::chrono::milliseconds timeout,
                       std::function<void(size_t trigger_count)> callback,
                       size_t max_trigger_times = 1,
                       std::chrono::milliseconds start_delay = std::chrono::milliseconds(0)) {
        if (topic.empty()) {
            ROS_ERROR("RegisterTopic failed: topic is empty");
            return false;
        }
        if (timeout.count() <= 0) {
            ROS_ERROR("RegisterTopic failed: invalid timeout %ldms for topic '%s'",
                     timeout.count(), topic.c_str());
            return false;
        }
        if (!callback) {
            ROS_ERROR("RegisterTopic failed: callback is null for topic '%s'", topic.c_str());
            return false;
        }
        if (max_trigger_times == 0) {
            ROS_WARN("RegisterTopic: topic '%s' set to infinite trigger times", topic.c_str());
        }

        if (start_delay.count() < 0) {
            ROS_ERROR("RegisterTopic failed: invalid start delay %ldms for topic '%s'",
                 start_delay.count(), topic.c_str());
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = monitors_.find(topic);
        if (it != monitors_.end()) {
            // 重复注册：完全重置状态，确保精准度
            it->second.last_received = Clock::now();
            it->second.last_triggered = Clock::now() - timeout;
            it->second.timeout = timeout;
            it->second.callback = std::move(callback);
            it->second.max_trigger_times = max_trigger_times;
            it->second.trigger_count.store(0);
            it->second.start_delay = start_delay;
            it->second.register_time = Clock::now(); 
            ROS_WARN("RegisterTopic: topic '%s' already exists, overwritten (reset all states)", topic.c_str());
        } else {
            monitors_.emplace(
                std::piecewise_construct,
                std::forward_as_tuple(topic),
                std::forward_as_tuple(
                    Clock::now(),
                    timeout,
                    std::move(callback),
                    max_trigger_times,
                    start_delay
                )
            );
            ROS_INFO("RegisterTopic success: topic '%s', timeout %ldms, max trigger times: %ld",
                    topic.c_str(), timeout.count(), max_trigger_times);
        }

        return true;
    }

    /**
     * @brief 复位话题（收到数据时调用）
     * 完全重置超时状态，确保下次触发逻辑精准
     */
    bool Reset(const std::string& topic) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = monitors_.find(topic);
        if (it == monitors_.end()) {
            ROS_WARN("Reset failed: topic '%s' not registered", topic.c_str());
            return false;
        }

        // 全量重置：接收时间、触发时间、触发次数
        it->second.last_received = Clock::now();
        it->second.last_triggered = Clock::now() - it->second.timeout;
        it->second.trigger_count.store(0);
        // ROS_DEBUG("Reset success: topic '%s' (all states reset)", topic.c_str());
        return true;
    }

    /**
     * @brief 注销话题
     */
    bool UnregisterTopic(const std::string& topic) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = monitors_.find(topic);
        if (it == monitors_.end()) {
            ROS_WARN("UnregisterTopic failed: topic '%s' not registered", topic.c_str());
            return false;
        }

        monitors_.erase(it);
        ROS_INFO("UnregisterTopic success: topic '%s'", topic.c_str());
        return true;
    }

    /**
     * @brief 获取当前监控的话题数量
     */
    size_t GetMonitorCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return monitors_.size();
    }

    /**
     * @brief 析构函数
     */
    ~TimeoutManager() {
        stop();
        ROS_INFO("TimeoutManager destroyed");
    }

private:
    /**
     * @brief 启动定时器（循环轮询）
     */
    void startTimer() {
        timer_.expires_after(CHECK_INTERVAL);
        timer_.async_wait([this](boost::system::error_code ec) {
            if (ec) {
                if (ec != boost::asio::error::operation_aborted) {
                    ROS_ERROR("Timer error: %s", ec.message().c_str());
                }
                return;
            }

            if (!is_running_.load()) {
                return;
            }

            checkTimeouts();
            startTimer();
        });
    }

    /**
     * @brief 超时检查核心逻辑（确保精准触发的关键）
     * 三层条件严格校验：1. 已超时 2. 触发间隔达标 3. 未达最大次数
     */
    void checkTimeouts() {
        auto now = Clock::now();
        std::vector<std::pair<std::function<void(size_t)>, size_t>> expired_callbacks;
        std::vector<std::string> expired_topics;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& pair : monitors_) {
                const std::string& topic = pair.first;
                MonitorInfo& info = pair.second;

                auto time_since_register = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - info.register_time);
                if (time_since_register < info.start_delay) {
                    continue;
                }

                // 基础条件：话题已超时（距离上次接收数据超过阈值）
                auto time_since_received = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - info.last_received);
                if (time_since_received <= info.timeout) {
                    continue;
                }

                // 间隔条件：距离上次触发超过阈值（避免频繁触发，核心精准度保障）
                auto time_since_triggered = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - info.last_triggered);
                if (time_since_triggered < info.timeout) {
                    continue;
                }

                // 次数条件：未达最大触发次数（0=无限次）
                size_t current_count = info.trigger_count.load();
                if (info.max_trigger_times != 0 && current_count >= info.max_trigger_times) {
                    continue;
                }

                // 满足所有条件：原子递增计数，更新触发时间
                size_t new_count = info.trigger_count.fetch_add(1) + 1;
                info.last_triggered = now; 

                // 加入回调队列（解锁后执行，避免阻塞轮询）
                expired_callbacks.emplace_back(info.callback, new_count);
                expired_topics.emplace_back(topic);

                // 日志：显示关键参数，方便调试精准度
                // ROS_WARN("Topic '%s' timeout triggered (count: %ld): "
                //         "since received: %ldms, since last trigger: %ldms, max times: %ld",
                //         topic.c_str(), new_count,
                //         time_since_received.count(),
                //         time_since_triggered.count(),
                //         info.max_trigger_times);
            }
        }

        for (size_t i = 0; i < expired_callbacks.size(); ++i) {
            try {
                std::function<void(size_t)>& cb = expired_callbacks[i].first;
                size_t count = expired_callbacks[i].second;
                cb(count);
            } catch (const std::exception& e) {
                ROS_ERROR("Timeout callback exception for topic '%s' (count: %ld): %s",
                        expired_topics[i].c_str(), expired_callbacks[i].second, e.what());
            } catch (...) {
                ROS_ERROR("Timeout callback unknown exception for topic '%s' (count: %ld)",
                        expired_topics[i].c_str(), expired_callbacks[i].second);
            }
        }
    }

    /**
     * @brief 停止管理器（析构时调用）
     */
    void stop() {
        if (is_running_.exchange(false)) {
            boost::system::error_code ec;
            timer_.cancel(ec);
            if (ec) {
                ROS_WARN("Timer cancel error: %s", ec.message().c_str());
            }

            io_context_.stop();

            if (io_thread_.joinable()) {
                io_thread_.join();
            }
        }
    }
};

} // namespace SystemMonitor