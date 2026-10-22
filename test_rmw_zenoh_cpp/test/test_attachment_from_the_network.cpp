// Copyright 2026 nevlife
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

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/strings.hpp>
#include <zenoh.hxx>

#include "rmw_zenoh_cpp/rmw_zenoh.hpp"

using namespace std::chrono_literals;

class TestAttachmentFromTheNetwork : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

// A peer, not necessarily rmw_zenoh_cpp of this version, controls the attachment. A count of
// broker exclusions that the bytes cannot hold must not take the subscriber down.
TEST_F(TestAttachmentFromTheNetwork, oversized_exclusion_count_is_ignored)
{
  auto node = std::make_shared<rclcpp::Node>("attachment_from_the_network");
  const std::string topic = "attachment_from_the_network";
  std::atomic<int> received{0};
  auto subscription = node->create_subscription<test_msgs::msg::Strings>(
    topic, 10, [&received](test_msgs::msg::Strings::ConstSharedPtr) {received++;});
  auto publisher = node->create_publisher<test_msgs::msg::Strings>(topic, 10);

  auto rmw_context = rcl_context_get_rmw_context(
    node->get_node_base_interface()->get_context()->get_rcl_context().get());
  auto session = rmw_zenoh_get_session(rmw_context);
  ASSERT_NE(session, nullptr);
  size_t domain_id = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_context_get_domain_id(
      node->get_node_base_interface()->get_context()->get_rcl_context().get(), &domain_id));

  // The key and payload of a real sample, to send a forged one next to it.
  std::mutex mutex;
  std::optional<std::string> key;
  std::optional<std::vector<uint8_t>> payload;
  auto spy = session->declare_subscriber(
    zenoh::KeyExpr(std::to_string(domain_id) + "/" + topic + "/**"),
    [&](const zenoh::Sample & sample) {
      std::lock_guard<std::mutex> lock(mutex);
      key = std::string(sample.get_keyexpr().as_string_view());
      payload = sample.get_payload().as_vector();
    },
    zenoh::closures::none);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_until = [&](auto && done) {
      const auto deadline = std::chrono::steady_clock::now() + 10s;
      while (!done() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(10ms);
        std::this_thread::sleep_for(10ms);
      }
      return done();
    };
  ASSERT_TRUE(spin_until([&] {
      if (received.load() == 0) {publisher->publish(test_msgs::msg::Strings());}
      std::lock_guard<std::mutex> lock(mutex);
      return received.load() > 0 && key.has_value();
    }));

  zenoh::ext::Serializer serializer;
  serializer.serialize(int64_t{1000});
  serializer.serialize(int64_t{0});
  serializer.serialize(std::array<uint8_t, RMW_GID_STORAGE_SIZE>{});
  // An exclusion list header claiming 2^40 entries and no entry after it.
  ASSERT_EQ(Z_OK, ::ze_serializer_serialize_sequence_length(
      zenoh::interop::as_loaned_c_ptr(serializer), size_t{1} << 40));
  zenoh::Session::PutOptions options;
  options.attachment = std::move(serializer).finish();
  std::vector<uint8_t> bytes;
  {
    std::lock_guard<std::mutex> lock(mutex);
    bytes = *payload;
  }
  const int before = received.load();
  session->put(zenoh::KeyExpr(*key), zenoh::Bytes(bytes), std::move(options));

  EXPECT_TRUE(spin_until([&] {return received.load() > before;}));
  publisher->publish(test_msgs::msg::Strings());
  EXPECT_TRUE(spin_until([&] {return received.load() > before + 1;}));
}
