#ifndef PSAF_UCBRIDGE_SYNCHRONIZEDQUEUE_H
#define PSAF_UCBRIDGE_SYNCHRONIZEDQUEUE_H

#include <string>
#include <deque>
#include <utility>
#include <mutex>
#include <memory>
#include <condition_variable>

/**
 * @short Synchronized queue for thread safe communication between one producer and one consumer thread
 * (not multiple pruducer/consumer)
 * @tparam T
 */

template<typename T>
class MessageQueue
{
public:
  using SharedPtr = std::shared_ptr<MessageQueue<T>>;

  /**
   * @short Gives back reference to the first element and removes reference from the underlying queue.
   * This blocks the thread until a new element is available.
   * @return reference to the first element
   */
  bool pop(
    T
    & data,
    const std::chrono::microseconds & timeout = standard_timeout,
    const bool discard = false
  )
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // if (queue_.empty()) {
    auto result = cond_.wait_for(
      lock, timeout,
      [this]() {
        return !queue_.empty();
      });
    if (!result) {
      return false;
    }
    // }
    if (discard) {
      while (queue_.size() > 1) {
        queue_.pop_front();
      }
    }
    data = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }
//  bool pop(
//      T & data
//      ){
//    std::unique_lock<std::mutex> lock(mutex_);
//    auto result = cond_.wait(lock,[this](){return !queue_.empty();});
//    if (!result) { return false;}
//    data = std::move(queue_.front());
//    queue_.pop_front();
//    return true;
//  }
  /**
   * @short Adds item to the queue
   * @param item
   */
  void push_back(const T & item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_back(item);
    lock.unlock();
    cond_.notify_one();
  }

  /**
   * @short Adds item to the queue
   * @param item
   */
  void push_back(T && item)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push_back(item);
    lock.unlock();
    cond_.notify_one();
  }

  /**
   * @short Checks if underlying data structure is empty
   * @return
   */
  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  /**
   * Wakes up all waiting threads
   */
  /*void notifyAll() {
    std::unique_lock<std::mutex> lock(mutex_);
    notified = true;
    cond_.notify_all();
  }*/
  size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:
  using QueueType = std::deque<T>;
  static constexpr std::chrono::microseconds standard_timeout =
    std::chrono::microseconds(100000);
  QueueType queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
  // bool notified = false;
};

#endif  // PSAF_UCBRIDGE_SYNCHRONIZEDQUEUE_H
