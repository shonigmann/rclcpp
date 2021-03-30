// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/loaned_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/empty.hpp"
#include "rclcpp/msg/string.hpp"


using namespace std::chrono_literals;

class TestPublisher : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

protected:
  void initialize(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns", node_options);
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

namespace rclcpp {

template <>
struct TypeAdapter<std::string, rclcpp::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = rclcpp::msg::String;

  static void
  convert_to_ros_message(
      const custom_type &source,
      ros_message_type &destination)
  {
    destination.data = source;
  }

  static void
  convert_to_custom(
      const ros_message_type &source,
      custom_type &destination)
  {
    destination = source.data;
  }
};

template <>
struct TypeAdapter<int, rclcpp::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = int;
  using ros_message_type = rclcpp::msg::String;

  static void
  convert_to_ros_message(
      const custom_type &source,
      ros_message_type &destination)
  {
    (void) source;
    (void) destination;
    throw std::runtime_error("This should happen");
  }

  static void
  convert_to_custom(
      const ros_message_type &source,
      custom_type &destination)
  {
    (void) source;
    (void) destination;
  }
};

}

/*
 * Testing publisher creation signatures with a type adapter.
 */
TEST_F(TestPublisher, various_creation_signatures) {
  for(auto is_intra_process : {true, false}) {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(is_intra_process);
    initialize(options);
    {
      using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
      auto publisher = node->create_publisher<StringTypeAdapter>("topic", 42);
      (void)publisher;
    }
    /* TODO(audrow) Enable this test once the adapt_type<>::as<> syntax is supported
    {
      using StringTypeAdapter = rclcpp::adapt_type<std::string>::as<rclcpp::msg::String>;
      auto publisher = node->create_publisher<StringTypeAdapter>("topic", 42);
      (void)publisher;
    }
    */
  }
}

/*
 * Testing that publisher sends type adapted types and ROS message types.
 */
#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

static const int max_loops = 200;
static const std::chrono::milliseconds sleep_per_loop(10);

class CLASSNAME (test_intra_process_within_one_node, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(CLASSNAME(test_intra_process_within_one_node, RMW_IMPLEMENTATION), nominal_usage) {

  // TODO(audrow) test all message types

  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;
  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";

  int counter = 0;
  auto callback =
    [message_data, &counter](
    const rclcpp::msg::String::SharedPtr msg,
    const rclcpp::MessageInfo & message_info
    ) -> void
    {
      counter++;
      ASSERT_STREQ(message_data.c_str(), msg->data.c_str());
      ASSERT_TRUE(message_info.get_rmw_message_info().from_intra_process);
    };

  auto node = rclcpp::Node::make_shared(
    "test_intra_process",
    rclcpp::NodeOptions().use_intra_process_comms(true));
  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 10);
  auto sub = node->create_subscription<rclcpp::msg::String>(topic_name, 1, callback);


  rclcpp::executors::SingleThreadedExecutor executor;
  {
    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    ASSERT_EQ(0, counter);
    printf("spin_node_some() - no callback expected\n");
    executor.spin_node_some(node);
    ASSERT_EQ(0, counter);

    // wait a moment for everything to initialize
    // TODO(gerkey): fix nondeterministic startup behavior
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    pub->publish(message_data);
    ASSERT_EQ(0, counter);

    // wait for the first callback
    {
      int i = 0;
      executor.spin_node_once(node, std::chrono::milliseconds(0));
      while (counter == 0 && i < max_loops) {
        printf("spin_node_once() - callback (1) expected - try %d/%d\n", ++i, max_loops);
        std::this_thread::sleep_for(sleep_per_loop);
        executor.spin_node_once(node, std::chrono::milliseconds(0));
      }
    }
    ASSERT_EQ(1, counter);
  }
}

/*
 * Testing that publisher sends type adapted types and ROS message types.
 */
TEST_F(TestPublisher, check_type_adapted_message_is_sent_and_received_inter_process) {
  using StringTypeAdapter = rclcpp::TypeAdapter<std::string, rclcpp::msg::String>;

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);
  initialize(options);

  const std::string message_data = "Message Data";
  const std::string topic_name = "topic_name";

  auto do_nothing = [](std::shared_ptr<const rclcpp::msg::String>) {FAIL();};
  auto pub = node->create_publisher<StringTypeAdapter>(topic_name, 1);
  auto sub = node->create_subscription<rclcpp::msg::String>(topic_name, 1, do_nothing);

  auto assert_no_message_was_received_yet = [sub](){
    rclcpp::msg::String msg;
    rclcpp::MessageInfo msg_info;
    EXPECT_FALSE(sub->take(msg, msg_info));
  };
  auto assert_message_was_received = [sub, message_data](){
    rclcpp::msg::String msg;
    rclcpp::MessageInfo msg_info;
    bool message_recieved = false;
    auto start = std::chrono::steady_clock::now();
    do {
      message_recieved = sub->take(msg, msg_info);
      std::this_thread::sleep_for(100ms);
    } while (!message_recieved && std::chrono::steady_clock::now() - start < 10s);
    EXPECT_TRUE(message_recieved);
    ASSERT_STREQ(message_data.c_str(), msg.data.c_str());
  };

  { // std::string passed by reference
    assert_no_message_was_received_yet();
    pub->publish(message_data);
    assert_message_was_received();
  }
  { // unique pointer to std::string
    assert_no_message_was_received_yet();
    auto pu_message = std::make_unique<std::string>(message_data);
    pub->publish(std::move(pu_message));
    assert_message_was_received();
  }
  { // ROS message passed by reference
    assert_no_message_was_received_yet();
    rclcpp::msg::String msg;
    msg.data = message_data;
    pub->publish(msg);
    assert_message_was_received();
  }
  { // unique ptr to ROS message
    assert_no_message_was_received_yet();
    auto pu_msg = std::make_unique<rclcpp::msg::String>();
    pu_msg->data = message_data;
    pub->publish(std::move(pu_msg));
    assert_message_was_received();
  }
  { // loaned ROS message
    assert_no_message_was_received_yet();
    std::allocator<void> allocator;
    rclcpp::LoanedMessage<rclcpp::msg::String> loaned_msg(*pub, allocator);
    loaned_msg.get().data = message_data;
    pub->publish(std::move(loaned_msg));
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    assert_message_was_received();
  }
}
/*
 * Testing that conversion errors are passed up.
 */
TEST_F(TestPublisher, conversion_exception_is_passed_up) {
  using BadStringTypeAdapter = rclcpp::TypeAdapter<int, rclcpp::msg::String>;
  for(auto is_intra_process : {true, false}) {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(is_intra_process);
    initialize(options);
    auto pub = node->create_publisher<BadStringTypeAdapter>("topic_name", 1);
    EXPECT_THROW(pub->publish(1), std::runtime_error);
  }
}