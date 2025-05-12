cmake_minimum_required(VERSION 3.8)
project(rob599_basic_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# If we're going to turn on the optimizer, we can add the -O3
# compiler option.
#if(CMAKE_COMPILER_IS_GNUCXX)
#  add_compile_options(-O3)
#endif()

# find dependencies.  We always need ament_cmake
find_package(ament_cmake REQUIRED)

# C++ packages need rclcpp.  Add in the other packages we need here, too.
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

# Our custom packages.
find_package(rob599_msgs REQUIRED)

# Add in the executables.  Each executable will need to be
# declared with add_executable, and the dependencies specified
# with ament_targe_dependencies.
add_executable(node src/node.cpp)
ament_target_dependencies(node rclcpp)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(publisher2 src/publisher2.cpp)
ament_target_dependencies(publisher2 rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(twice src/twice.cpp)
ament_target_dependencies(twice rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp rob599_msgs)

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp rob599_msgs)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action rob599_msgs)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action rob599_msgs)


# Make sure we install the nodes so that ros2 run can find them.
install(TARGETS
  node
  publisher
  publisher2
  subscriber
  twice

  service_server
  service_client

  action_server
  action_client

  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files so that ros2 launch can find them.
install(DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
