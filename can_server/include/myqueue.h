#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iostream>
#include <thread>

enum class QueueResult {
    kSuccess,
    kClosed,
    kTimeout
};

template<typename T>

class Myqueue{
    private:
        std::queue<T> queue_;
        std::size_t capacity_;
        std::mutex mutex_;
        std::condition_variable not_full_;
        std::condition_variable not_empty_;
        bool closed_;
    public:
        explicit Myqueue(std::size_t capacity) : capacity_(capacity) , closed_(false) {}

        void close(){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                closed_ = true;
            }
            not_full_.notify_all();
            not_empty_.notify_all();
        }
        template<typename Rep, typename Period>
        QueueResult push(T value,const std::chrono::duration<Rep,Period>& timeout){
            std::unique_lock<std::mutex> lock(mutex_);
            bool ok = not_full_.wait_for(lock, timeout ,[this] { return queue_.size() < capacity_ || closed_ ;});
            if(!ok){
                return QueueResult::kTimeout;
            }
            if(closed_){
                return QueueResult::kClosed;
            }
            queue_.push(std::move(value));
            not_empty_.notify_one();
            return QueueResult::kSuccess;
        }
        template<typename Rep, typename Period>
        QueueResult pop(T& value,const std::chrono::duration<Rep,Period>& timeout){
            std::unique_lock<std::mutex> lock(mutex_);
            bool ok = not_empty_.wait_for(lock, timeout ,[this] { return !queue_.empty() || closed_; });
            if(!ok){
                return QueueResult::kTimeout;
            }
            if(queue_.empty()){
                return QueueResult::kClosed;
            }

            value = std::move(queue_.front());
            queue_.pop();
            not_full_.notify_one();
            return QueueResult::kSuccess;
        }
        bool is_closed(){
            std::lock_guard<std::mutex>lock(mutex_);
            return closed_;
        }

};