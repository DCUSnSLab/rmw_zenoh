// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef DETAIL__LOANED_MESSAGE_HPP_
#define DETAIL__LOANED_MESSAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include <zenoh.hxx>

#include "rcutils/allocator.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

namespace rmw_zenoh_cpp
{
struct LoanType
{
  static std::unique_ptr<LoanType> make(
    const rosidl_message_type_support_t * typesupport,
    const message_type_support_callbacks_t * callbacks);

  size_t size;
  size_t alignment;
  std::function<void(void *)> init;
  std::function<void(void *)> fini;
};

class LoanBuffer
{
public:
  static std::unique_ptr<LoanBuffer> allocate(
    const LoanType & type, rcutils_allocator_t allocator,
    const zenoh::ShmProvider * provider = nullptr);

  ~LoanBuffer();

  void * data() const;
  zenoh::Bytes release_payload();

private:
  explicit LoanBuffer(const LoanType & type, rcutils_allocator_t allocator);

  const LoanType & type_;
  rcutils_allocator_t allocator_;
  std::optional<zenoh::ZShmMut> shm_;
  uint8_t * data_;
  std::optional<zenoh::Bytes> payload_;
};
}  // namespace rmw_zenoh_cpp

#endif  // DETAIL__LOANED_MESSAGE_HPP_
