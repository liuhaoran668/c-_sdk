#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include "linkerhand/exceptions.hpp"

namespace linkerhand {

class StopIteration : public std::runtime_error {
 public:
  StopIteration() : std::runtime_error("Queue is closed and empty") {}
};

template <typename T>
class IterableQueue {
 private:
  struct State {
    explicit State(std::size_t maxsize_) : maxsize(maxsize_) {}

    const std::size_t maxsize;
    std::mutex mutex;
    std::condition_variable cv_not_empty;
    std::condition_variable cv_not_full;
    std::deque<T> queue;
    bool closed = false;
  };

 public:
  explicit IterableQueue(std::size_t maxsize = 0)
      : state_(std::make_shared<State>(maxsize)) {}

  /// Blocking put. Waits until space is available. Throws StateError if closed.
  void put(const T& item) { put_blocking(state_, item); }
  void put(T&& item) { put_blocking(state_, std::move(item)); }

  /// Non-blocking put. Returns false if queue is full or closed.
  bool try_put(const T& item) { return try_put_impl(state_, item); }
  bool try_put(T&& item) { return try_put_impl(state_, std::move(item)); }

  /// Drops the oldest item if the queue is full. Returns false only if the queue is closed.
  bool force_put(T item) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->closed) {
      return false;
    }
    const bool bounded = state_->maxsize > 0;
    if (bounded && state_->queue.size() >= state_->maxsize) {
      state_->queue.pop_front();
    }
    state_->queue.push_back(std::move(item));
    state_->cv_not_empty.notify_one();
    return true;
  }

  /// Blocking get. Waits until an item is available. Throws StopIteration if closed and empty.
  T get() { return get_blocking(state_); }

  /// Blocking get with timeout. Returns nullopt on timeout. Throws StopIteration if closed and empty.
  std::optional<T> get_for(std::chrono::milliseconds timeout) {
    return get_timed(state_, timeout);
  }

  /// Non-blocking get. Returns nullopt if empty.
  std::optional<T> try_get() {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->queue.empty()) {
      return std::nullopt;
    }
    T item = std::move(state_->queue.front());
    state_->queue.pop_front();
    state_->cv_not_full.notify_one();
    return item;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->queue.empty();
  }

  void close() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->closed = true;
    state_->cv_not_empty.notify_all();
    state_->cv_not_full.notify_all();
  }

  class Iterator {
   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = const T*;
    using reference = const T&;

    Iterator() = default;

    explicit Iterator(std::shared_ptr<State> state) : state_(std::move(state)) {
      advance();
    }

    reference operator*() const { return *current_; }
    pointer operator->() const { return &(*current_); }

    Iterator& operator++() {
      advance();
      return *this;
    }

    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const Iterator& a, const Iterator& b) {
      return a.state_ == b.state_;
    }

    friend bool operator!=(const Iterator& a, const Iterator& b) { return !(a == b); }

   private:
    void advance() {
      if (!state_) {
        return;
      }

      try {
        current_ = get_blocking(state_);
      } catch (const StopIteration&) {
        state_.reset();
        current_.reset();
      }
    }

    std::shared_ptr<State> state_;
    std::optional<T> current_;
  };

  Iterator begin() { return Iterator(state_); }
  Iterator end() { return Iterator(); }

  Iterator begin() const { return Iterator(state_); }
  Iterator end() const { return Iterator(); }

 private:
  template <typename U>
  static void put_blocking(const std::shared_ptr<State>& state, U&& item) {
    std::unique_lock<std::mutex> lock(state->mutex);
    if (state->closed) {
      throw StateError("Cannot put to a closed queue");
    }

    const bool bounded = state->maxsize > 0;
    if (!bounded || state->queue.size() < state->maxsize) {
      state->queue.push_back(std::forward<U>(item));
      lock.unlock();
      state->cv_not_empty.notify_one();
      return;
    }

    state->cv_not_full.wait(lock, [&] { return state->queue.size() < state->maxsize || state->closed; });

    if (state->closed) {
      throw StateError("Cannot put to a closed queue");
    }

    state->queue.push_back(std::forward<U>(item));
    lock.unlock();
    state->cv_not_empty.notify_one();
  }

  template <typename U>
  static bool try_put_impl(const std::shared_ptr<State>& state, U&& item) {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (state->closed) {
      return false;
    }
    const bool bounded = state->maxsize > 0;
    if (bounded && state->queue.size() >= state->maxsize) {
      return false;
    }
    state->queue.push_back(std::forward<U>(item));
    state->cv_not_empty.notify_one();
    return true;
  }

  static T get_blocking(const std::shared_ptr<State>& state) {
    std::unique_lock<std::mutex> lock(state->mutex);
    state->cv_not_empty.wait(lock, [&] { return !state->queue.empty() || state->closed; });

    if (state->queue.empty()) {
      throw StopIteration();
    }

    T item = std::move(state->queue.front());
    state->queue.pop_front();
    lock.unlock();
    state->cv_not_full.notify_one();
    return item;
  }

  static std::optional<T> get_timed(const std::shared_ptr<State>& state,
                                    std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(state->mutex);

    if (state->closed && state->queue.empty()) {
      throw StopIteration();
    }

    if (!state->cv_not_empty.wait_for(lock, timeout,
                                       [&] { return !state->queue.empty() || state->closed; })) {
      return std::nullopt;
    }

    if (state->closed && state->queue.empty()) {
      throw StopIteration();
    }

    if (state->queue.empty()) {
      return std::nullopt;
    }

    T item = std::move(state->queue.front());
    state->queue.pop_front();
    lock.unlock();
    state->cv_not_full.notify_one();
    return item;
  }

  std::shared_ptr<State> state_;
};

}  // namespace linkerhand
