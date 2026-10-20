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

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cinttypes>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "test_rmw_zenoh_cpp/msg/plain_payload.hpp"

using Plain = test_rmw_zenoh_cpp::msg::PlainPayload;
using Image = sensor_msgs::msg::Image;

void reset_copies()
{
  auto reset = reinterpret_cast<void (*)()>(dlsym(RTLD_DEFAULT, "memcount_reset"));
  if (reset) {
    reset();
  }
}

int64_t stamp(const Plain & msg) {return msg.stamp;}
int64_t stamp(const Image & msg) {return rclcpp::Time(msg.header.stamp).nanoseconds();}
void stamp(Plain & msg, int64_t ns) {msg.stamp = ns;}
void stamp(Image & msg, int64_t ns) {msg.header.stamp = rclcpp::Time(ns);}
void prepare(Plain &) {}
void prepare(Image & msg)
{
  msg.encoding = "mono8";
  msg.width = 4194304;
  msg.height = 1;
  msg.step = msg.width;
  msg.data.resize(msg.width);
}

template<typename Message>
void run(const rclcpp::Node::SharedPtr & node, bool publisher)
{
  const auto count = node->declare_parameter<int>("count", 120);
  const auto warmup = node->declare_parameter<int>("warmup", 10);
  const auto hz = node->declare_parameter<double>("hz", 30.0);
  if (publisher) {
    const auto subscribers = node->declare_parameter<int>("subscribers", 1);
    auto pub = node->create_publisher<Message>("loan_benchmark", rclcpp::QoS(5));
    std::printf("publisher can_loan=%d\n", pub->can_loan_messages());
    while (rclcpp::ok() && pub->get_subscription_count() < static_cast<size_t>(subscribers)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto next = std::chrono::steady_clock::now();
    for (int i = -warmup; i < count; ++i) {
      if (i == 0) {
        reset_copies();
      }
      auto msg = pub->borrow_loaned_message();
      prepare(msg.get());
      std::memset(msg.get().data.data(), (i + warmup) & 255, msg.get().data.size());
      stamp(msg.get(), node->now().nanoseconds());
      pub->publish(std::move(msg));
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / hz));
      std::this_thread::sleep_until(next);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  } else {
    const auto hold_ms = node->declare_parameter<int>("hold_ms", 0);
    int received = 0;
    uint64_t checksum = 0;
    std::vector<double> latencies;
    auto sub = node->create_subscription<Message>(
      "loan_benchmark", rclcpp::QoS(5), [&](const Message & msg) {
        if (++received <= warmup) {
          if (received == warmup) {
            reset_copies();
          }
          if (received == 1 && hold_ms > 0) {
            // The publisher keeps writing new loans while this one is held.
            const auto first = msg.data.front();
            std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
            const bool intact = std::all_of(
              msg.data.begin(), msg.data.end(), [first](auto b) {return b == first;});
            std::printf(
              "held %d ms: %s\n", static_cast<int>(hold_ms), intact ? "intact" : "changed");
          }
          return;
        }
        latencies.push_back((node->now().nanoseconds() - stamp(msg)) / 1000.0);
        unsigned char value = 0;
        for (auto byte : msg.data) {
          value |= byte;
        }
        checksum += value;
      });
    std::printf("subscription can_loan=%d\n", sub->can_loan_messages());
    while (rclcpp::ok() && received < count + warmup) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    std::sort(latencies.begin(), latencies.end());
    std::printf("received=%zu checksum=%" PRIu64 " p50_us=%.1f p99_us=%.1f\n",
      latencies.size(), checksum, latencies.at(latencies.size() / 2),
      latencies.at(latencies.size() * 99 / 100));
  }
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(std::string("loan_benchmark_") + argv[2]);
  auto invoke = [&](const rclcpp::Node::SharedPtr & n, bool pub) {
      if (std::string(argv[1]) == "plain") {
        run<Plain>(n, pub);
      } else {
        run<Image>(n, pub);
      }
    };
  if (std::string(argv[2]) == "both") {
    auto receiver = std::make_shared<rclcpp::Node>("loan_benchmark_receiver");
    std::thread subscriber([&]() {invoke(receiver, false);});
    invoke(node, true);
    subscriber.join();
  } else {
    invoke(node, std::string(argv[2]) == "pub");
  }
  rclcpp::shutdown();
}
