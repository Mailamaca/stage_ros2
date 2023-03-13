cmake_minimum_required(VERSION 3.12)
project(stage_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # About -Wno-sign-conversion: With Clang, -Wconversion implies -Wsign-conversion. There are a number of
  # implicit sign conversions in rclcpp and gtest.cc, see https://ci.ros2.org/job/ci_osx/9265/.
  # Hence disabling -Wsign-conversion for now until all those have eventually been fixed.
  # (from https://github.com/ros2/rclcpp/pull/1188#issuecomment-650229140)
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(stage REQUIRED)

# build
add_executable(stage_ros2 src/stage_ros2.cc)
target_include_directories(stage_ros2
    PRIVATE ${STAGE_INCLUDE_DIRS} include)
target_link_libraries(stage_ros2
    PRIVATE ${STAGE_LIBRARIES})

# Add dependencies on standard ROS 2 packages using ament.
ament_target_dependencies(stage_ros2
    PUBLIC
    rclcpp
    rcutils
    cv_bridge
    image_transport
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    stage
    tf2_geometry_msgs)

install(TARGETS stage_ros2
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY stage
    DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()


