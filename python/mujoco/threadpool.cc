// Copyright 2024 DeepMind Technologies Limited
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

#include "threadpool.h"

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

#include <absl/base/attributes.h>

namespace mujoco::python {

ABSL_CONST_INIT thread_local int ThreadPool::worker_id_ = -1;

// ThreadPool constructor
ThreadPool::ThreadPool(int num_threads) : ctr_(0) {
  for (int i = 0; i < num_threads; i++) {
    threads_.push_back(std::thread(&ThreadPool::WorkerThread, this, i));
  }
}

// ThreadPool destructor
ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(m_);
    for (int i = 0; i < threads_.size(); i++) {
      queue_.push(nullptr);
    }
    cv_in_.notify_all();
  }
  for (auto& thread : threads_) {
    thread.join();
  }
}

// ThreadPool scheduler
void ThreadPool::Schedule(std::function<void()> task) {
  std::unique_lock<std::mutex> lock(m_);
  queue_.push(std::move(task));
  cv_in_.notify_one();
}

// ThreadPool worker
void ThreadPool::WorkerThread(int i) {
  worker_id_ = i;
  while (true) {
    auto task = [&]() {
      std::unique_lock<std::mutex> lock(m_);
      cv_in_.wait(lock, [&]() { return !queue_.empty(); });
      std::function<void()> task = std::move(queue_.front());
      queue_.pop();
      cv_in_.notify_one();
      return task;
    }();
    if (task == nullptr) {
      {
        std::unique_lock<std::mutex> lock(m_);
        ++ctr_;
        cv_ext_.notify_one();
      }
      break;
    }
    task();

    {
      std::unique_lock<std::mutex> lock(m_);
      ++ctr_;
      cv_ext_.notify_one();
    }
  }
}

}  // namespace mujoco::python
