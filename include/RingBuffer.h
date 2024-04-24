#pragma once

#include <cstdint>
#include <mutex>
#include <memory>

class RingBuffer {
private:
    std::unique_ptr<uint8_t[]> buffer;
    uint32_t size;
    uint32_t capacity;
    uint32_t front;
    uint32_t rear;
    bool full;
    mutable std::mutex mtx;

public:
    RingBuffer(uint32_t capacity = 512);
    ~RingBuffer();

    bool isEmpty() const;
    bool isFull() const;
    uint32_t getSize() const;
    uint32_t getCapacity() const;

    bool pushBack(const uint8_t* data, uint32_t length); // Modified function signature
    bool popFront(uint8_t* data, uint32_t length);       // Modified function signature
};


