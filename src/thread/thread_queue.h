// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_SRC_THREAD_LOCKLESS_QUEUE_H_
#define MUJOCO_SRC_THREAD_LOCKLESS_QUEUE_H_

#include <atomic>
#include <climits>
#include <cstddef>
#include <thread>

namespace mujoco {

// A Lockless Queue allows for sending information quickly between different
// threads. This is a Multi-Producer Multi-Consumer Lockless Queue allowing for
// multiple threads to be adding items to the queue while multiple threads are
// consuming items from the queue. Internally it uses a Ring Buffer for storage
// so it will not grow as items are added. Push will block if the Queue is full
// and Pop will block if it is empty.
//
// For a basic overview of this category of structures:
// https://www.linuxjournal.com/content/lock-free-multi-producer-multi-consumer-queue-ring-buffer
template  <typename T, size_t buffer_capacity>
class LocklessQueue {
 public:
  bool full() const {
    return full_internal(
        convert_to_index(read_cursor_), convert_to_index(write_cursor_));
  }

  bool empty() const {
    return maximum_read_cursor_ == read_cursor_;
  }

  // Push an element into the queue.
  void push(const T& input) {
    // Reserve a slot in the queue
    size_t current_write_cursor;
    size_t dummy_current_write_cursor;
    size_t next_write_cursor;
    size_t current_write_index;
    size_t current_read_index;
    do {
      // Check if the queue is full.
      do {
        current_write_cursor = write_cursor_.load();
        current_write_index = convert_to_index(current_write_cursor);
        next_write_cursor = get_next_cursor(current_write_cursor);

        current_read_index = convert_to_index(read_cursor_.load());
      } while (full_internal(current_read_index, current_write_index));

      // Once it's not full, attempt to grab a slot to write.
      dummy_current_write_cursor = current_write_cursor;
    } while (!write_cursor_.compare_exchange_weak(
                 dummy_current_write_cursor, next_write_cursor));

    // Write the entry.
    buffer_[current_write_index].store(input);

    // Increment maximum read cursor. Note here it has to wait if the compare
    // and exchange fails as another thread might not have completed its write.
    do {
      dummy_current_write_cursor = current_write_cursor;
    } while (!maximum_read_cursor_.compare_exchange_weak(
                 dummy_current_write_cursor, next_write_cursor));
  }

  // Pop an element from the queue.
  T pop() {
    size_t current_read_cursor;
    size_t dummy_current_read_cursor;
    size_t current_read_index;
    size_t next_read_cursor;
    size_t current_maximum_read_cursor;
    size_t current_maximum_read_index;
    bool empty = false;
    T result;
    do {
      // Wait until the queue has an element
      do {
        if (empty) {
          std::this_thread::yield();
        }
        current_read_cursor = read_cursor_.load();
        current_maximum_read_cursor = maximum_read_cursor_.load();

        current_read_index = convert_to_index(current_read_cursor);
        current_maximum_read_index = convert_to_index(
            current_maximum_read_cursor);

        empty = empty_internal(
            current_read_index, current_maximum_read_index);
      } while (empty);

      next_read_cursor = get_next_cursor(current_read_cursor);

      // Attempt to grab the element, if unsuccessful then wait for the next
      // element to arrive.
      result = buffer_[current_read_index].load();
      dummy_current_read_cursor = current_read_cursor;
    } while (!read_cursor_.compare_exchange_weak(
                 dummy_current_read_cursor, next_read_cursor));

    return result;
  }

 private:
  size_t convert_to_index(size_t input) const {
    return input % internal_buffer_capacity_;
  }

  size_t get_next_cursor(size_t input) const {
    return (input + 1) % cursor_max_;
  }

  size_t get_next_index(size_t input) const {
    return convert_to_index(get_next_cursor(input));
  }

  bool full_internal(size_t read_index, size_t write_index) const {
    return get_next_index(write_index) == read_index;
  }

  bool empty_internal(size_t read_index, size_t write_index) const {
    return read_index == write_index;
  }

  const size_t internal_buffer_capacity_ = buffer_capacity + 1;
  const size_t cursor_max_ = UINT_MAX - (UINT_MAX % internal_buffer_capacity_);

  std::atomic<size_t> read_cursor_ = 0;
  std::atomic<size_t> write_cursor_ = 0;
  std::atomic<size_t> maximum_read_cursor_ = 0;

  std::atomic<T> buffer_[(buffer_capacity + 1)];
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_THREAD_LOCKLESS_QUEUE_H_
