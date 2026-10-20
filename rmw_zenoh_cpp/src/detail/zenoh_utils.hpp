// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__ZENOH_UTILS_HPP_
#define DETAIL__ZENOH_UTILS_HPP_

#include <zenoh.hxx>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "rcutils/allocator.h"
#include "rcutils/env.h"
#include "rmw/types.h"
#include "logging_macros.hpp"

namespace rmw_zenoh_cpp
{
///=============================================================================
// A class to store the replies to service requests.
class ZenohReply final
{
public:
  ZenohReply(const zenoh::Reply & reply, std::chrono::nanoseconds::rep received_timestamp);

  ~ZenohReply() = default;

  const zenoh::Reply & get_sample() const;

  std::chrono::nanoseconds::rep get_received_timestamp() const;

private:
  std::optional<zenoh::Reply> reply_;
  std::chrono::nanoseconds::rep received_timestamp_;
};

// A class to store the queries made by clients.
///=============================================================================
class ZenohQuery final
{
public:
  ZenohQuery(const zenoh::Query & query, std::chrono::nanoseconds::rep received_timestamp);

  ~ZenohQuery() = default;

  const zenoh::Query & get_query() const;

  std::chrono::nanoseconds::rep get_received_timestamp() const;

private:
  zenoh::Query query_;
  std::chrono::nanoseconds::rep received_timestamp_;
};

///=============================================================================
int64_t get_system_time_in_ns();

///=============================================================================
class Payload
{
public:
  explicit Payload(const zenoh::Bytes & bytes);

  ~Payload() = default;

  // Null when the payload is a CDR header and body in separate slices.
  const uint8_t * data() const;

  size_t size() const;

  bool empty() const;

  const uint8_t * contiguous_data(size_t offset, size_t length) const;

  void copy_to(uint8_t * destination) const;

private:
  zenoh::Bytes bytes_;
  std::optional<std::vector<uint8_t>> flat_;
};

///=============================================================================
struct ShmContext
{
  size_t msgsize_threshold;

  explicit ShmContext(size_t msgsize_threshold, size_t pool_size);

  std::optional<zenoh::SharedShmProvider> get_shm_provider(zenoh::Session & session);

  size_t pool_size() const;

  // Created on first use: zenoh locks the whole pool in memory.
  std::shared_ptr<zenoh::PosixShmProvider> get_loan_provider();

private:
  size_t pool_size_;
  std::mutex loan_mutex_;
  std::shared_ptr<zenoh::PosixShmProvider> loan_provider_;
};

///=============================================================================
class BufferPool
{
public:
  class Buffer
  {
    friend class BufferPool;

public:
    uint8_t * data;
    size_t size;

private:
    Buffer(uint8_t * data_ptr, size_t size_val)
    : data(data_ptr), size(size_val) {}
  };

  BufferPool();

  ~BufferPool();

  std::optional<Buffer> allocate(size_t size);

  void deallocate(Buffer buffer);

private:
  std::vector<Buffer> buffers_;
  std::mutex mutex_;
  size_t max_size_;
  size_t size_;
  // NOTE(fuzzypixelz): Pooled buffers are recycled with the expectation that they would reside in
  // cache, thus this this value should be comparable to the size of a modern CPU cache. The default
  // value (8 MiB) is relatively conservative as CPU cache sizes range from a few MiB to a few
  // hundred MiB.
  const size_t DEFAULT_MAX_SIZE = 8 * 1024 * 1024;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__ZENOH_UTILS_HPP_
