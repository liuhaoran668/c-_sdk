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
#include <type_traits>

#include "linkerhand/exceptions.hpp"

namespace linkerhand {

class QueueEmpty : public std::runtime_error {
 public:
  QueueEmpty() : std::runtime_error("Queue is empty") {}
};

class QueueFull : public std::runtime_error {
 public:
  QueueFull() : std::runtime_error("Queue is full") {}
};

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

  void put(const T& item, bool block = true, std::optional<double> timeout_s = std::nullopt) {
    put_impl(state_, item, block, timeout_s);
  }

  void put_nowait(const T& item) { put(item, /*block=*/false, std::nullopt); }

  T get(bool block = true, std::optional<double> timeout_s = std::nullopt) {
    return get_impl(state_, block, timeout_s);
  }

  T get_nowait() { return get(/*block=*/false, std::nullopt); }

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
        current_ = get_impl(state_, /*block=*/true, std::nullopt);
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
  static void put_impl(
      const std::shared_ptr<State>& state,
      const T& item,
      bool block,
      const std::optional<double>& timeout_s) {
    std::unique_lock<std::mutex> lock(state->mutex);
    if (state->closed) {
      throw StateError("Cannot put to a closed queue");
    }

    const bool bounded = state->maxsize > 0;
    if (!bounded) {
      state->queue.push_back(item);
      lock.unlock();
      state->cv_not_empty.notify_one();
      return;
    }

    auto has_space = [&]() { return state->queue.size() < state->maxsize; };
    if (has_space()) {
      state->queue.push_back(item);
      lock.unlock();
      state->cv_not_empty.notify_one();
      return;
    }

    if (!block) {
      throw QueueFull();
    }

    auto has_space_or_closed = [&]() { return has_space() || state->closed; };

    if (timeout_s.has_value()) {
      const auto timeout =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(*timeout_s));
      if (!state->cv_not_full.wait_for(lock, timeout, has_space_or_closed)) {
        throw QueueFull();
      }
    } else {
      state->cv_not_full.wait(lock, has_space_or_closed);
    }

    if (state->closed) {
      throw StateError("Cannot put to a closed queue");
    }

    state->queue.push_back(item);
    lock.unlock();
    state->cv_not_empty.notify_one();
  }

  static T get_impl(
      const std::shared_ptr<State>& state,
      bool block,
      const std::optional<double>& timeout_s) {
    std::unique_lock<std::mutex> lock(state->mutex);

    if (state->closed && state->queue.empty()) {
      throw StopIteration();
    }

    auto has_item_or_closed = [&]() { return !state->queue.empty() || state->closed; };

    if (!block) {
      if (state->queue.empty()) {
        throw QueueEmpty();
      }
      T item = std::move(state->queue.front());
      state->queue.pop_front();
      lock.unlock();
      state->cv_not_full.notify_one();
      return item;
    }

    if (timeout_s.has_value()) {
      const auto timeout =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(*timeout_s));
      if (!state->cv_not_empty.wait_for(lock, timeout, has_item_or_closed)) {
        throw QueueEmpty();
      }
    } else {
      state->cv_not_empty.wait(lock, has_item_or_closed);
    }

    if (state->closed && state->queue.empty()) {
      throw StopIteration();
    }

    if (state->queue.empty()) {
      throw QueueEmpty();
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
