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

#include "loaned_message.hpp"

#include <algorithm>
#include <cstring>
#include <utility>
#include <variant>

#include "rmw/error_handling.h"
#include "rosidl_runtime_c/message_initialization.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rmw_zenoh_cpp
{
namespace
{
size_t primitive_size(uint8_t type)
{
  switch (type) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return 1;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return 2;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return 4;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return 8;
    default:
      return 0;
  }
}

template<typename Members>
bool compatible_layout(const Members & members, size_t base, size_t & offset, size_t & alignment)
{
  if (members.member_count_ == 0) {
    return false;
  }
  for (uint32_t i = 0; i < members.member_count_; ++i) {
    const auto & field = members.members_[i];
    if (field.is_array_ && (field.is_upper_bound_ || field.array_size_ == 0)) {
      return false;
    }
    const size_t count = field.is_array_ ? field.array_size_ : 1;
    if (field.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      const auto & nested = *static_cast<const Members *>(field.members_->data);
      const size_t nested_base = base + field.offset_;
      size_t nested_alignment = 1;
      if (!compatible_layout(nested, nested_base, offset, nested_alignment)) {
        return false;
      }
      alignment = std::max(alignment, nested_alignment);
      offset += (count - 1) * nested.size_of_;
    } else {
      const size_t size = primitive_size(field.type_id_);
      if (size == 0) {
        return false;
      }
      alignment = std::max(alignment, size);
      offset = (offset + size - 1) & ~(size - 1);
      if (offset != base + field.offset_) {
        return false;
      }
      offset += count * size;
    }
  }
  // CDR omits trailing struct padding, including between array elements.
  return offset == base + members.size_of_;
}

zenoh::AllocAlignment shm_alignment(size_t alignment)
{
  uint8_t power = 0;
  while ((size_t{1} << power) < alignment) {
    ++power;
  }
  return zenoh::AllocAlignment{power};
}
}  // namespace

std::unique_ptr<LoanType> LoanType::make(
  const rosidl_message_type_support_t * typesupport,
  const message_type_support_callbacks_t * callbacks)
{
  size_t serialized_size = 0;
#ifdef ROSIDL_TYPESUPPORT_FASTRTPS_HAS_PLAIN_TYPES
  char bounds;
  serialized_size = callbacks->max_serialized_size(bounds);
  if (bounds != ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE) {
    return nullptr;
  }
#else
  (void)typesupport;
  (void)callbacks;
  return nullptr;
#endif
  auto type = std::make_unique<LoanType>();
  type->alignment = 1;
  size_t offset = 0;
  const auto * support = get_message_typesupport_handle(
    typesupport, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  if (support) {
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(support->data);
    if (!compatible_layout(*members, 0, offset, type->alignment)) {
      return nullptr;
    }
    type->size = members->size_of_;
    type->init = [members](void * data) {
        members->init_function(data, rosidl_runtime_cpp::MessageInitialization::ALL);
      };
    type->fini = members->fini_function;
  } else {
    rmw_reset_error();
    support = get_message_typesupport_handle(
      typesupport, rosidl_typesupport_introspection_c__identifier);
    if (!support) {
      rmw_reset_error();
      return nullptr;
    }
    const auto * members =
      static_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(support->data);
    if (!compatible_layout(*members, 0, offset, type->alignment)) {
      return nullptr;
    }
    type->size = members->size_of_;
    type->init = [members](void * data) {
        members->init_function(data, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
      };
    type->fini = members->fini_function;
  }
  if (type->size != serialized_size) {
    return nullptr;
  }
  return type;
}

LoanBuffer::LoanBuffer(const LoanType & type, rcutils_allocator_t allocator)
: type_(type), allocator_(allocator), data_(nullptr)
{}

std::unique_ptr<LoanBuffer> LoanBuffer::allocate(
  const LoanType & type, rcutils_allocator_t allocator, const zenoh::ShmProvider * provider)
{
  auto buffer = std::unique_ptr<LoanBuffer>(new LoanBuffer(type, allocator));
  if (provider) {
    auto allocation = provider->alloc_gc_defrag(type.size, shm_alignment(type.alignment));
    if (std::holds_alternative<zenoh::ZShmMut>(allocation)) {
      buffer->shm_ = std::get<zenoh::ZShmMut>(std::move(allocation));
      buffer->data_ = buffer->shm_->data();
    }
  }
  if (!buffer->data_) {
    buffer->data_ = static_cast<uint8_t *>(allocator.allocate(type.size, allocator.state));
  }
  if (!buffer->data_) {
    return nullptr;
  }
  // Zero padding as well as fields before exposing the representation on the wire.
  std::memset(buffer->data_, 0, type.size);
  type.init(buffer->data_);
  return buffer;
}

LoanBuffer::~LoanBuffer()
{
  if (data_) {
    type_.fini(data_);
    if (!shm_ && !payload_) {
      allocator_.deallocate(data_, allocator_.state);
    }
  }
}

void * LoanBuffer::data() const
{
  return data_;
}

zenoh::Bytes LoanBuffer::release_payload()
{
  if (payload_) {
    return payload_->clone();
  }
  uint8_t * data = data_;
  zenoh::Bytes body;
  if (shm_) {
    body = zenoh::Bytes(std::move(*shm_));
  } else {
    body = zenoh::Bytes(data, type_.size, [allocator = allocator_](uint8_t * ptr) {
          allocator.deallocate(ptr, allocator.state);
        });
  }
  const uint16_t native = 1;
  const uint8_t header[4] = {0, *reinterpret_cast<const uint8_t *>(&native), 0, 0};
  zenoh::Bytes::Writer writer;
  writer.write_all(header, sizeof(header));
  writer.append(std::move(body));
  payload_ = std::move(writer).finish();
  return payload_->clone();
}
}  // namespace rmw_zenoh_cpp
