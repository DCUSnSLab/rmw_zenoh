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

#include "zenoh_utils.hpp"

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstring>
#include <memory>
#include <utility>

#include "attachment_helpers.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rmw/error_handling.h"

namespace rmw_zenoh_cpp
{
///=============================================================================
ZenohQuery::ZenohQuery(
  const zenoh::Query & query,
  std::chrono::nanoseconds::rep received_timestamp)
: query_(query.clone())
{
  received_timestamp_ = received_timestamp;
}

///=============================================================================
std::chrono::nanoseconds::rep ZenohQuery::get_received_timestamp() const
{
  return received_timestamp_;
}

///=============================================================================
const zenoh::Query & ZenohQuery::get_query() const {return query_;}

///=============================================================================
ZenohReply::ZenohReply(
  const zenoh::Reply & reply,
  std::chrono::nanoseconds::rep received_timestamp)
{
  reply_ = reply.clone();
  received_timestamp_ = received_timestamp;
}

///=============================================================================
const zenoh::Reply & ZenohReply::get_sample() const
{
  return reply_.value();
}

///=============================================================================
std::chrono::nanoseconds::rep ZenohReply::get_received_timestamp() const
{
  return received_timestamp_;
}

int64_t get_system_time_in_ns()
{
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

///=============================================================================
std::vector<uint8_t> flatten(const zenoh::Bytes & bytes)
{
  std::vector<uint8_t> flat;
  flat.reserve(bytes.size());
  auto slices = bytes.slice_iter();
  while (auto slice = slices.next()) {
    flat.insert(flat.end(), slice->data, slice->data + slice->len);
  }
  return flat;
}

///=============================================================================
Payload::Payload(const zenoh::Bytes & bytes)
{
  // A loaning publisher sends the CDR header and the aligned body as two slices. Keep those so
  // that the body is read in place; flatten anything else once, on the receiving thread.
  auto slices = bytes.slice_iter();
  auto first = slices.next();
  if (first && slices.next() && (first->len != 4 || slices.next())) {
    flat_ = flatten(bytes);
  } else {
    bytes_ = bytes.clone();
  }
}

///=============================================================================
const uint8_t * Payload::contiguous_data(size_t offset, size_t length) const
{
  if (flat_) {
    return offset + length <= flat_->size() ? flat_->data() + offset : nullptr;
  }
  auto slices = bytes_.slice_iter();
  while (auto slice = slices.next()) {
    if (offset < slice->len) {
      return length <= slice->len - offset ? slice->data + offset : nullptr;
    }
    offset -= slice->len;
  }
  return nullptr;
}

///=============================================================================
const uint8_t * Payload::data() const
{
  return contiguous_data(0, size());
}

///=============================================================================
size_t Payload::size() const
{
  return flat_ ? flat_->size() : bytes_.size();
}

///=============================================================================
bool Payload::empty() const
{
  return size() == 0;
}

///=============================================================================
void Payload::copy_to(uint8_t * destination) const
{
  if (flat_) {
    memcpy(destination, flat_->data(), flat_->size());
    return;
  }
  auto slices = bytes_.slice_iter();
  while (auto slice = slices.next()) {
    memcpy(destination, slice->data, slice->len);
    destination += slice->len;
  }
}

///=============================================================================
ShmContext::ShmContext(size_t msgsize_threshold, size_t pool_size)
: msgsize_threshold(msgsize_threshold), pool_size_(pool_size)
{}

std::optional<zenoh::SharedShmProvider> ShmContext::get_shm_provider(zenoh::Session & session)
{
  auto maybe_provider = session.obtain_shm_provider();
  if (std::holds_alternative<zenoh::SharedShmProvider>(maybe_provider)) {
    return std::get<zenoh::SharedShmProvider>(std::move(maybe_provider));
  }
  return std::nullopt;
}


size_t ShmContext::pool_size() const
{
  return pool_size_;
}

std::shared_ptr<zenoh::PosixShmProvider> ShmContext::get_loan_provider()
{
  std::lock_guard<std::mutex> lock(loan_mutex_);
  if (!loan_provider_ && !loan_provider_failed_) {
    // The transport provider only supports byte alignment. ROS structs require up to eight.
    zenoh::ZResult result;
    zenoh::MemoryLayout layout((pool_size_ + 7) & ~size_t{7}, zenoh::AllocAlignment{3}, &result);
    std::shared_ptr<zenoh::PosixShmProvider> provider;
    if (result == Z_OK) {
      provider = std::make_shared<zenoh::PosixShmProvider>(layout, &result);
    }
    if (result != Z_OK) {
      // Retrying costs a pool sized mapping and mlock on every borrow, e.g. under a low
      // RLIMIT_MEMLOCK, so loans fall back to the heap for the rest of the process.
      loan_provider_failed_ = true;
      RMW_ZENOH_LOG_WARN_NAMED(
        "rmw_zenoh_cpp",
        "Unable to create the %zu byte shared memory pool for loaned messages, "
        "loans use the heap", pool_size_);
      return nullptr;
    }
    loan_provider_ = std::move(provider);
  }
  return loan_provider_;
}


///=============================================================================
BufferPool::BufferPool()
: buffers_(), mutex_()
{
  const char * env_value;
  const char * error_str = rcutils_get_env("RMW_ZENOH_BUFFER_POOL_MAX_SIZE_BYTES", &env_value);
  if (error_str != nullptr) {
    RMW_ZENOH_LOG_WARN_NAMED(
      "rmw_zenoh_cpp",
      "Unable to read maximum buffer pool size, falling back to default.");
    max_size_ = DEFAULT_MAX_SIZE;
  } else if (strcmp(env_value, "") == 0) {
    max_size_ = DEFAULT_MAX_SIZE;
  } else {
    max_size_ = std::atoll(env_value);
  }
  size_ = 0;
}

///=============================================================================
BufferPool::~BufferPool()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  for (Buffer & buffer : buffers_) {
    allocator.deallocate(buffer.data, allocator.state);
  }
}

///=============================================================================
std::optional<BufferPool::Buffer> BufferPool::allocate(size_t size)
{
  std::lock_guard<std::mutex> guard(mutex_);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (buffers_.empty()) {
    if (size_ + size > max_size_) {
      return std::nullopt;
    } else {
      size_ += size;
    }
    uint8_t * data = static_cast<uint8_t *>(allocator.allocate(size, allocator.state));
    if (data == nullptr) {
      return std::nullopt;
    } else {
      return Buffer(data, size);
    }
  } else {
    Buffer buffer = buffers_.back();
    buffers_.pop_back();
    if (buffer.size < size) {
      const size_t size_diff = size - buffer.size;
      if (size_ + size_diff > max_size_) {
        return std::nullopt;
      }
      uint8_t * data = static_cast<uint8_t *>(allocator.reallocate(
          buffer.data, size, allocator.state));
      if (data == nullptr) {
        return std::nullopt;
      }
      size_ += size_diff;
      buffer.data = data;
      buffer.size = size;
    }
    return buffer;
  }
}

///=============================================================================
void BufferPool::deallocate(BufferPool::Buffer buffer)
{
  std::lock_guard<std::mutex> guard(mutex_);
  buffers_.push_back(buffer);
}

}  // namespace rmw_zenoh_cpp
