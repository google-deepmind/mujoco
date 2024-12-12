// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MJPC_THREADPOOL_H_
#define MJPC_THREADPOOL_H_

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <absl/base/attributes.h>

namespace mjpc {

// ThreadPool class
class ThreadPool {
 public:
  // constructor
  explicit ThreadPool(int num_threads);

  // destructor
  ~ThreadPool();

  int NumThreads() const { return threads_.size(); }

  // returns an ID between 0 and NumThreads() - 1. must be called within
  // worker thread (returns -1 if not).
  static int WorkerId() { return worker_id_; }

  // ----- methods ----- //
  // set task for threadpool
  void Schedule(std::function<void()> task);

  // return number of tasks completed
  std::uint64_t GetCount() { return ctr_; }

  // reset count to zero
  void ResetCount() { ctr_ = 0; }

  // wait for count, then return
  void WaitCount(int value) {
    std::unique_lock<std::mutex> lock(m_);
    cv_ext_.wait(lock, [&]() { return this->GetCount() >= value; });
  }

 private:
  // ----- methods ----- //

  // execute task with available thread
  void WorkerThread(int i);

  ABSL_CONST_INIT static thread_local int worker_id_;

  // ----- members ----- //
  std::vector<std::thread> threads_;
  std::mutex m_;
  std::condition_variable cv_in_;
  std::condition_variable cv_ext_;
  std::queue<std::function<void()>> queue_;
  std::uint64_t ctr_;
};

}  // namespace mjpc

#endif  // MJPC_THREADPOOL_H_
