cmake_minimum_required(VERSION 3.8)
project(jetson_obs_distance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(jetson_obs_distance src/jetson_obs_distance.cpp src/platform.cpp src/vl53l5cx_api.cpp)

ament_target_dependencies(jetson_obs_distance rclcpp std_msgs sensor_msgs px4_msgs)
target_include_directories(jetson_obs_distance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jetson_obs_distance PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS jetson_obs_distance
  DESTINATION lib/${PROJECT_NAME})

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
