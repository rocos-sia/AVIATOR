#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

namespace aviator {
struct TargetTraceFrame {
    uint64_t mono_us{}, callback{}, loop_epoch{};
    std::array<double, 7> target{}, measured{}, velocity{};
};
// One SDK producer, one serialized executor consumer. No allocation, locks,
// formatting or file IO on push(). A full queue drops NEW frames, never races
// by overwriting a frame that the consumer is reading.
class TargetTrace {
public:
    static constexpr size_t queue_capacity = 4096, history_capacity = 256;
    static constexpr size_t capture_capacity = queue_capacity + history_capacity;
    TargetTrace() { frames_.reserve(capture_capacity); }
    void push(const TargetTraceFrame& frame) noexcept {
        const auto head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) == queue_capacity) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        queue_[head % queue_capacity] = frame;
        head_.store(head + 1, std::memory_order_release);
    }
    void poll() {
        auto tail = tail_.load(std::memory_order_relaxed);
        const auto head = head_.load(std::memory_order_acquire);
        while (tail != head) {
            const auto frame = queue_[tail % queue_capacity];
            history_[history_count_++ % history_capacity] = frame;
            if (armed_ && frames_.size() < capture_capacity && frame.mono_us <= trigger_us_ + 1000000)
                frames_.push_back(frame);
            ++tail;
        }
        tail_.store(tail, std::memory_order_release);
    }
    // First request of each enable session; never silently overwrite evidence.
    bool begin(uint64_t now, const std::array<double, 7>& target) {
        poll();
        if (armed_) return false;
        trigger_us_ = now;
        expected_ = target;
        const auto first = history_count_ > history_capacity ? history_count_ - history_capacity : 0;
        for (auto i = first; i < history_count_; ++i)
            frames_.push_back(history_[i % history_capacity]);
        armed_ = true;
        return true;
    }
    // Only after the producer has stopped.
    void reset() {
        head_ = 0; tail_ = 0; dropped_ = 0;
        armed_ = false; history_count_ = 0; trigger_us_ = 0; frames_.clear();
    }
    bool armed() const { return armed_; }
    uint64_t trigger() const { return trigger_us_; }
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
    bool full() const { return frames_.size() == capture_capacity; }
    const auto& expected() const { return expected_; }
    const auto& frames() const { return frames_; }
private:
    static_assert(std::atomic<uint64_t>::is_always_lock_free);
    std::array<TargetTraceFrame, queue_capacity> queue_{};
    std::atomic<uint64_t> head_{0}, tail_{0}, dropped_{0};
    std::array<TargetTraceFrame, history_capacity> history_{};
    uint64_t history_count_ = 0, trigger_us_ = 0;
    bool armed_ = false;
    std::array<double, 7> expected_{};
    std::vector<TargetTraceFrame> frames_;
};
}
