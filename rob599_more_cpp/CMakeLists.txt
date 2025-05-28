cmake_minimum_required(VERSION 3.8)
project(rob599_more_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# If we're going to turn on the optimizer, we can add the -O3
# compiler option.
#if(CMAKE_COMPILER_IS_GNUCXX)
#  add_compile_options(-O3)
#endif()

# Tell the compiler where to find the include files.
include_directories(include)

# find dependencies.  We always need ament_cmake
find_package(ament_cmake REQUIRED)

# C++ packages need rclcpp.  Add in the other packages we need here, too.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# We need this to make the PCL stuff work.
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add in the executables.
add_executable(cloud_generator src/cloud_generator_node.cpp src/cloud_generator.cpp)
ament_target_dependencies(cloud_generator rclcpp sensor_msgs)

add_executable(cloud_timer src/cloud_timer_node.cpp src/cloud_timer.cpp)
ament_target_dependencies(cloud_timer rclcpp sensor_msgs)

add_executable(cloud_combined
  src/cloud_combined.cpp
  src/cloud_generator_node.cpp
  src/cloud_timer_node.cpp
)
ament_target_dependencies(cloud_combined rclcpp sensor_msgs)

add_executable(cylinders src/cylinders.cpp)
target_link_libraries(cylinders ${PCL_LIBRARIES})
ament_target_dependencies(cylinders rclcpp sensor_msgs pcl_ros pcl_conversions)

# Make sure we install the nodes so that ros2 run can find them.
install(TARGETS
  cloud_generator
  cloud_timer
  cloud_combined
  cylinders

  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files so that ros2 launch can find them.
#install(DIRECTORY
#  launch
#
#  DESTINATION share/${PROJECT_NAME}
#)

# Install the include files so that other pacakges can find them.
install(DIRECTORY
  include

  DESTINATION include/${PROJECT_NAME}
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
