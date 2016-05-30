/*
 * Copyright (c) 2013 - 2016 Stefan Muller Arisona, Simon Schubiger
 * Copyright (c) 2013 - 2016 FHNW
 * All rights reserved.
 *
 * Contributions by: Filip Schramka, Samuel von Stachelski
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *  Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *  Neither the name of FHNW nor the names of its contributors may be used to
 *   endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <utility>

namespace Avion {
    
enum class QueueTakeResult {
    ok,
    overflow,
    timeout
};

template <typename T, typename C = std::deque<T>>
class BlockingQueue final {

    const int maxSize;
    std::mutex mutex;
    std::condition_variable condition;
    C queue;
    std::atomic_bool overflow{false};

    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;
    BlockingQueue(BlockingQueue &&) = delete;
    
public:
    explicit BlockingQueue(int maxSize) : maxSize(maxSize) {}
    
    bool put(const T& value) {
        if (overflow)
            return false;
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.size < maxSize) {
            queue.push_front(value);
            lock.unlock();
            condition.notify_one();
            return true;
        } else {
            queue.clear();
            overflow = true;
            return false;
        }
    }
    
    QueueTakeResult take(T& value) {
        if (overflow.exchange(false))
            return QueueTakeResult::overflow;
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [=]{ return !queue.empty(); });
        value = std::move(queue.back());
        queue.pop_back();
        return QueueTakeResult::ok;
    }
    
    QueueTakeResult take(T& value, std::chrono::milliseconds ms) {
        if (overflow.exchange(false))
            return QueueTakeResult::overflow;
        std::unique_lock<std::mutex> lock(mutex);
        if (!condition.wait_for(lock, ms, [=]{ return !queue.empty(); }))
            return QueueTakeResult::timeout;
        value = std::move(queue.back());
        queue.pop_back();
        return QueueTakeResult::ok;
    }
    
    int size() const {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.size();
    }
};



template<typename T>
class AudioBuffer final {
    struct Entry {
        Entry(double pts, T* samples, int length) : pts(pts), length(length), start(0) {
            samples = new T[length];
            std::copy(samples, samples + length, samples);
        }

        ~Entry() {
            delete[] samples;
        }

        double pts;
        T* samples;
        int length;
        int start;
    };

    const double sampleRate;

    std::deque<std::shared_ptr<Entry>> queue;

    int bufferSize;

    AudioBuffer(const AudioBuffer&) = delete;
    AudioBuffer& operator=(const AudioBuffer&) = delete;
    AudioBuffer(AudioBuffer &&) = delete;

public:
    AudioBuffer(double sampleRate) : sampleRate(sampleRate), bufferSize(0) {
    }

    ~AudioBuffer() {
    }

    int size() const {
        return bufferSize;
    }

    void put(T* src, int num, double pts) {
        bufferSize += num;
        queue.push_back(std::make_shared<Entry>(pts, src, num));
    }

    int take(T* dst, int num, double& pts) {
        int result = 0;
        if (num > bufferSize)
            num = bufferSize;
        result = num;

        Entry* entry = queue.front().get();
        pts = entry->pts + (double)entry->start / sampleRate;
        while (num > 0) {
            int l = std::min(num, entry->length - entry->start);
            std::copy(entry->samples + entry->start, entry->samples + entry->start + l, dst);
            entry->start += l;
            if (entry->start == entry->length) {
                queue.pop_front();
                if (queue.empty())
                    break;
                entry = queue.front().get();
            }
            dst += l;
            num -= l;

        }

        bufferSize -= result;
        return result;
    }
    
    void clear() {
        queue.clear();
    }
};

}
