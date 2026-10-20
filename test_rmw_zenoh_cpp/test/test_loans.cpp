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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "rmw/error_handling.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "test_rmw_zenoh_cpp/msg/nested_payload.hpp"
#include "test_rmw_zenoh_cpp/msg/padded_payload.hpp"
#include "test_rmw_zenoh_cpp/msg/plain_payload.hpp"
#include "test_rmw_zenoh_cpp/msg/plain_leaf.h"

using Plain = test_rmw_zenoh_cpp::msg::PlainPayload;
using Nested = test_rmw_zenoh_cpp::msg::NestedPayload;
using Padded = test_rmw_zenoh_cpp::msg::PaddedPayload;

class TestLoans : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_loans");
    pub = node->create_publisher<Plain>("loans", 5);
    sub = node->create_subscription<Plain>("loans", 5, [](const Plain &) {});
    ASSERT_TRUE(pub->can_loan_messages());
    ASSERT_TRUE(sub->can_loan_messages());
  }

  void TearDown() override
  {
    sub.reset();
    pub.reset();
    node.reset();
    rclcpp::shutdown();
    rmw_reset_error();
  }

  const rmw_publisher_t * publisher()
  {
    return rcl_publisher_get_rmw_handle(pub->get_publisher_handle().get());
  }

  const rmw_subscription_t * subscription()
  {
    return rcl_subscription_get_rmw_handle(sub->get_subscription_handle().get());
  }

  const rosidl_message_type_support_t * support()
  {
    return rosidl_typesupport_cpp::get_message_type_support_handle<Plain>();
  }

  void * take(rmw_message_info_t * info = nullptr)
  {
    void * message = nullptr;
    bool taken = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      auto ret = info ? rmw_take_loaned_message_with_info(
        subscription(), &message, &taken, info, nullptr) :
        rmw_take_loaned_message(subscription(), &message, &taken, nullptr);
      EXPECT_EQ(RMW_RET_OK, ret);
      if (taken) {
        return message;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    ADD_FAILURE() << "loaned take timed out";
    return nullptr;
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<Plain>::SharedPtr pub;
  rclcpp::Subscription<Plain>::SharedPtr sub;
};

TEST_F(TestLoans, round_trip_and_lifetime)
{
  void * borrowed = nullptr;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_message(publisher(), support(), &borrowed));
  auto * sent = static_cast<Plain *>(borrowed);
  EXPECT_EQ(0, sent->stamp);
  EXPECT_EQ(0, sent->data.back());
  sent->stamp = 12345;
  sent->data.fill(63);
  ASSERT_EQ(RMW_RET_OK, rmw_publish_loaned_message(publisher(), borrowed, nullptr));
  rmw_message_info_t info{};
  void * received = take(&info);
  ASSERT_NE(nullptr, received);
  EXPECT_EQ(borrowed, received);
  auto * message = static_cast<Plain *>(received);
  EXPECT_EQ(12345, message->stamp);
  const auto correct_byte = [](uint8_t b) {return b == 63;};
  EXPECT_TRUE(std::all_of(message->data.begin(), message->data.end(), correct_byte));
  EXPECT_GT(info.source_timestamp, 0);
  EXPECT_GT(info.received_timestamp, 0);
  EXPECT_EQ(1u, info.publication_sequence_number);
  EXPECT_FALSE(info.from_intra_process);
  rmw_gid_t gid{};
  ASSERT_EQ(RMW_RET_OK, rmw_get_gid_for_publisher(publisher(), &gid));
  EXPECT_EQ(0, std::memcmp(gid.data, info.publisher_gid.data, RMW_GID_STORAGE_SIZE));
  for (int i = 0; i < 20; ++i) {
    auto next = pub->borrow_loaned_message();
    EXPECT_NE(received, &next.get());
    next.get().data.fill(static_cast<uint8_t>(i));
    pub->publish(std::move(next));
  }
  EXPECT_EQ(63, message->data.front());
  EXPECT_EQ(63, message->data.back());
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(subscription(), received));
}

TEST_F(TestLoans, return_and_invalid_arguments)
{
  void * borrowed = nullptr;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_message(publisher(), support(), &borrowed));
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_borrow_loaned_message(publisher(), support(), &borrowed));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_return_loaned_message_from_subscription(
      subscription(), borrowed));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_publisher(publisher(), borrowed));
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT,
    rmw_return_loaned_message_from_publisher(publisher(), borrowed));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_publish_loaned_message(publisher(), borrowed, nullptr));
  rmw_reset_error();
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_borrow_loaned_message(nullptr, support(), &borrowed));
  rmw_reset_error();
  bool taken = true;
  void * received = nullptr;
  EXPECT_EQ(RMW_RET_OK, rmw_take_loaned_message(subscription(), &received, &taken, nullptr));
  EXPECT_FALSE(taken);
  EXPECT_EQ(nullptr, received);
  EXPECT_EQ(RMW_RET_INVALID_ARGUMENT, rmw_take_loaned_message_with_info(
      subscription(), &received, &taken, nullptr, nullptr));
  rmw_reset_error();
}

TEST_F(TestLoans, mixed_loaned_and_regular_takes)
{
  auto regular = node->create_subscription<Plain>("loans", 5, [](const Plain &) {});
  auto loan = pub->borrow_loaned_message();
  loan.get().stamp = 99;
  loan.get().data.fill(17);
  auto * original = &loan.get();
  pub->publish(std::move(loan));
  void * received = take();
  ASSERT_NE(nullptr, received);
  EXPECT_EQ(original, received);
  auto output = std::make_unique<Plain>();
  bool taken = false;
  auto * handle = rcl_subscription_get_rmw_handle(regular->get_subscription_handle().get());
  ASSERT_EQ(RMW_RET_OK, rmw_take(handle, output.get(), &taken, nullptr));
  ASSERT_TRUE(taken);
  EXPECT_EQ(99, output->stamp);
  EXPECT_EQ(17, output->data.back());
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(subscription(), received));
}

TEST_F(TestLoans, serialized_take_from_loaned_publisher)
{
  auto message = pub->borrow_loaned_message();
  message.get().stamp = 123;
  message.get().data.fill(87);
  pub->publish(std::move(message));
  rclcpp::SerializedMessage bytes;
  bool taken = false;
  ASSERT_EQ(RMW_RET_OK, rmw_take_serialized_message(
      subscription(), &bytes.get_rcl_serialized_message(), &taken, nullptr));
  ASSERT_TRUE(taken);
  rclcpp::Serialization<Plain> serializer;
  auto output = std::make_unique<Plain>();
  serializer.deserialize_message(&bytes, output.get());
  EXPECT_EQ(123, output->stamp);
  EXPECT_EQ(87, output->data.back());
}

TEST_F(TestLoans, regular_publisher_and_opposite_endian)
{
  auto message = std::make_unique<Plain>();
  message->stamp = 0x123456789abcdef;
  message->data.fill(42);
  pub->publish(*message);
  void * received = take();
  ASSERT_NE(nullptr, received);
  EXPECT_EQ(message->stamp, static_cast<Plain *>(received)->stamp);
  EXPECT_EQ(42, static_cast<Plain *>(received)->data.back());
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(subscription(), received));
  rclcpp::Serialization<Plain> serializer;
  rclcpp::SerializedMessage bytes;
  serializer.serialize_message(message.get(), &bytes);
  auto & serialized = bytes.get_rcl_serialized_message();
  serialized.buffer[1] ^= 1;
  std::reverse(serialized.buffer + 4, serialized.buffer + 12);
  ASSERT_EQ(RMW_RET_OK, rmw_publish_serialized_message(publisher(), &serialized, nullptr));
  received = take();
  ASSERT_NE(nullptr, received);
  EXPECT_EQ(message->stamp, static_cast<Plain *>(received)->stamp);
  EXPECT_EQ(42, static_cast<Plain *>(received)->data.back());
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(subscription(), received));
}

TEST_F(TestLoans, nested_types_defaults_and_padding)
{
  auto nested = node->create_publisher<Nested>("nested", 5);
  ASSERT_TRUE(nested->can_loan_messages());
  auto loan = nested->borrow_loaned_message();
  EXPECT_EQ(42u, loan.get().leaves[0].value);
  EXPECT_EQ(7u, loan.get().leaves[2].tag);
  EXPECT_EQ(0u, loan.get().values[1]);
  auto padded = node->create_publisher<Padded>("padded", 5);
  EXPECT_FALSE(padded->can_loan_messages());
  auto image = node->create_publisher<sensor_msgs::msg::Image>("image", 5);
  EXPECT_FALSE(image->can_loan_messages());
}

TEST_F(TestLoans, c_typesupport)
{
  auto * rmw_node = rcl_node_get_rmw_handle(
    node->get_node_base_interface()->get_shared_rcl_node_handle().get());
  auto options = rmw_get_default_publisher_options();
  auto * handle = rmw_create_publisher(
    rmw_node, ROSIDL_GET_MSG_TYPE_SUPPORT(test_rmw_zenoh_cpp, msg, PlainLeaf),
    "/c_loan", &rmw_qos_profile_default, &options);
  ASSERT_NE(nullptr, handle);
  EXPECT_TRUE(handle->can_loan_messages);
  void * message = nullptr;
  ASSERT_EQ(RMW_RET_OK, rmw_borrow_loaned_message(
      handle, ROSIDL_GET_MSG_TYPE_SUPPORT(test_rmw_zenoh_cpp, msg, PlainLeaf), &message));
  EXPECT_EQ(42u, static_cast<test_rmw_zenoh_cpp__msg__PlainLeaf *>(message)->value);
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_publisher(handle, message));
  EXPECT_EQ(RMW_RET_OK, rmw_destroy_publisher(rmw_node, handle));
}

TEST_F(TestLoans, unchanged_rclcpp_and_intra_process)
{
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto local = std::make_shared<rclcpp::Node>("intra_loans", options);
  auto publisher = local->create_publisher<Plain>("intra_loans", 5);
  EXPECT_FALSE(publisher->can_loan_messages());
  bool received = false;
  auto subscription = local->create_subscription<Plain>(
    "intra_loans", 5, [&](const Plain & message) {
      EXPECT_EQ(77, message.stamp);
      received = true;
    });
  auto message = publisher->borrow_loaned_message();
  message.get().stamp = 77;
  publisher->publish(std::move(message));
  rclcpp::spin_some(local);
  EXPECT_TRUE(received);
}

TEST_F(TestLoans, pool_exhaustion_falls_back)
{
  std::vector<rclcpp::LoanedMessage<Plain>> held;
  held.reserve(20);
  for (int i = 0; i < 20; ++i) {
    held.emplace_back(pub->borrow_loaned_message());
    EXPECT_TRUE(held.back().is_valid());
    held.back().get().stamp = i;
    EXPECT_EQ(0u, reinterpret_cast<uintptr_t>(&held.back().get()) % alignof(Plain));
  }
  pub->publish(std::move(held.back()));
  held.pop_back();
  void * received = take();
  ASSERT_NE(nullptr, received);
  EXPECT_EQ(19, static_cast<Plain *>(received)->stamp);
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(subscription(), received));
}

TEST_F(TestLoans, transient_local_and_publisher_destruction)
{
  auto qos = rclcpp::QoS(2).transient_local();
  auto cached = node->create_publisher<Plain>("cached_loans", qos);
  auto message = cached->borrow_loaned_message();
  auto * original = &message.get();
  message.get().stamp = 123;
  message.get().data.fill(91);
  cached->publish(std::move(message));
  auto late = node->create_subscription<Plain>("cached_loans", qos, [](const Plain &) {});
  auto * handle = rcl_subscription_get_rmw_handle(late->get_subscription_handle().get());
  void * received = nullptr;
  bool taken = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!taken && std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(RMW_RET_OK, rmw_take_loaned_message(handle, &received, &taken, nullptr));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(taken);
  EXPECT_EQ(original, received);
  cached.reset();
  EXPECT_EQ(123, static_cast<Plain *>(received)->stamp);
  EXPECT_EQ(91, static_cast<Plain *>(received)->data.back());
  EXPECT_EQ(RMW_RET_OK, rmw_return_loaned_message_from_subscription(handle, received));
}

class TestNoLoans : public ::testing::Test {};

TEST_F(TestNoLoans, capability_and_fallback)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("test_no_loans");
    auto pub = node->create_publisher<Plain>("no_loans", 5);
    auto sub = node->create_subscription<Plain>("no_loans", 5, [](const Plain &) {});
    EXPECT_FALSE(pub->can_loan_messages());
    EXPECT_FALSE(sub->can_loan_messages());
    auto message = pub->borrow_loaned_message();
    EXPECT_TRUE(message.is_valid());
    pub->publish(std::move(message));
    auto * handle = rcl_publisher_get_rmw_handle(pub->get_publisher_handle().get());
    void * output = nullptr;
    EXPECT_EQ(RMW_RET_UNSUPPORTED, rmw_borrow_loaned_message(
        handle, rosidl_typesupport_cpp::get_message_type_support_handle<Plain>(), &output));
    rmw_reset_error();
  }
  rclcpp::shutdown();
}
