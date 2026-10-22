cmake_minimum_required(VERSION 3.20)
project(test_rmw_zenoh_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcpputils REQUIRED)
  find_package(rmw_zenoh_cpp REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(sensor_msgs REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PlainPayload.msg"
    "msg/PlainLeaf.msg"
    "msg/NestedPayload.msg"
    "msg/PaddedPayload.msg")
  rosidl_get_typesupport_target(loan_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
  rosidl_get_typesupport_target(loan_typesupport_c ${PROJECT_NAME} rosidl_typesupport_c)

  add_executable(loan_benchmark test/loan_benchmark.cpp)
  target_link_libraries(loan_benchmark rclcpp::rclcpp ${loan_typesupport} ${sensor_msgs_TARGETS} ${CMAKE_DL_LIBS})
  find_package(zenoh_cpp_vendor REQUIRED)

  ament_add_gtest_executable(test_loans test/test_loans.cpp)
  target_link_libraries(test_loans
    rclcpp::rclcpp rmw_zenoh_cpp::rmw_zenoh_cpp ${loan_typesupport} ${loan_typesupport_c}
    ${sensor_msgs_TARGETS})
  ament_add_gtest_test(test_loans
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DISABLE_LOANED_MESSAGES=0
      ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=true
      GTEST_FILTER=TestLoans.*:TestLoanPool.*)
  ament_add_gtest_test(test_loans TEST_NAME test_loans_disabled
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DISABLE_LOANED_MESSAGES=0
      ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=false GTEST_FILTER=TestNoLoans.*)
  ament_add_gtest_test(test_loans TEST_NAME test_loans_undersized
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DISABLE_LOANED_MESSAGES=0
      ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=true
      ZENOH_SHM_ALLOC_SIZE=1048576 GTEST_FILTER=TestSmallPool.*)

  ament_add_gtest_test(test_loans TEST_NAME test_loans_broker
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DISABLE_LOANED_MESSAGES=0
      RCL_INTRA_PROCESS=1 ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=true
      GTEST_FILTER=TestBrokerLoans.*)

  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_rmw_zenoh_session
    test/test_rmw_zenoh_session.cpp
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp)
  target_link_libraries(test_rmw_zenoh_session
    rclcpp::rclcpp
    rmw_zenoh_cpp::rmw_zenoh_cpp
    zenohcxx::zenohc
  )

  ament_add_ros_isolated_gtest(test_attachment_from_the_network
    test/test_attachment_from_the_network.cpp
    TIMEOUT 60
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp RCL_INTRA_PROCESS=0)
  target_link_libraries(test_attachment_from_the_network
    rclcpp::rclcpp
    rmw_zenoh_cpp::rmw_zenoh_cpp
    zenohcxx::zenohc
    ${test_msgs_TARGETS}
  )

  ament_add_ros_isolated_gtest(test_client_destroy_during_reply
    test/test_client_destroy_during_reply.cpp
    TIMEOUT 120
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp)
  target_link_libraries(test_client_destroy_during_reply
    rclcpp::rclcpp
    rmw_zenoh_cpp::rmw_zenoh_cpp
    rcpputils::rcpputils
    ${test_msgs_TARGETS}
  )

  ament_add_ros_isolated_gtest(test_service_destroy_during_request
    test/test_service_destroy_during_request.cpp
    TIMEOUT 120
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp)
  target_link_libraries(test_service_destroy_during_request
    rclcpp::rclcpp
    rmw_zenoh_cpp::rmw_zenoh_cpp
    rcpputils::rcpputils
    ${test_msgs_TARGETS}
  )

  ament_add_ros_isolated_gtest(test_subscription_destroy_during_sample
    test/test_subscription_destroy_during_sample.cpp
    TIMEOUT 120
    ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp)
  target_link_libraries(test_subscription_destroy_during_sample
    rclcpp::rclcpp
    rmw_zenoh_cpp::rmw_zenoh_cpp
    rcpputils::rcpputils
    ${test_msgs_TARGETS}
  )
endif()

ament_package()
