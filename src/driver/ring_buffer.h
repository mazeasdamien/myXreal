#pragma once
#include <atomic>
#include <cstddef>
#include <array>

// Lock-free single-producer single-consumer ring buffer.
template <typename T, size_t Capacity>
class RingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");
    static constexpr size_t kMask = Capacity - 1;

    std::array<T, Capacity> buf_{};
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};

public:
    bool push(const T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);
        size_t next = (tail + 1) & kMask;
        if (next == head_.load(std::memory_order_acquire)) return false;
        buf_[tail] = item;
        tail_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        if (head == tail_.load(std::memory_order_acquire)) return false;
        item = buf_[head];
        head_.store((head + 1) & kMask, std::memory_order_release);
        return true;
    }

    void clear() {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    size_t available() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_acquire);
        if (t >= h) return t - h;
        return Capacity - h + t;
    }
};
