#include "RingBuffer.h"

RingBuffer::RingBuffer(uint32_t capacity) : size(0), capacity(capacity), front(0), rear(0), full(false) {
    buffer = std::make_unique<uint8_t[]>(capacity);
}

RingBuffer::~RingBuffer() = default;

bool RingBuffer::isEmpty() const {
    std::lock_guard<std::mutex> lock(mtx);
    return size == 0;
}

bool RingBuffer::isFull() const {
    std::lock_guard<std::mutex> lock(mtx);
    return full;
}

uint32_t RingBuffer::getSize() const {
    std::lock_guard<std::mutex> lock(mtx);
    return size;
}

uint32_t RingBuffer::getCapacity() const {
    std::lock_guard<std::mutex> lock(mtx);
    return capacity;
}

bool RingBuffer::pushBack(const uint8_t* data, uint32_t length) {
    std::lock_guard<std::mutex> lock(mtx);
    if (length > capacity - size) {
        return false; // Not enough space in the buffer
    }

    for (uint32_t i = 0; i < length; ++i) {
        buffer[rear] = data[i];
        rear = (rear + 1) % capacity;
    }

    size += length;

    if (size == capacity) {
        full = true;
    }

    return true;
}

bool RingBuffer::popFront(uint8_t* data, uint32_t length) {
    std::lock_guard<std::mutex> lock(mtx);
    if (length > size) {
        return false; // Not enough data in the buffer
    }

    for (uint32_t i = 0; i < length; ++i) {
        data[i] = buffer[front];
        front = (front + 1) % capacity;
    }

    size -= length;
    full = false;

    return true;
}
