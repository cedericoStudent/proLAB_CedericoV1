cmake_minimum_required(VERSION 3.5)
project(tb4_cedric_pkg_1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Installiere die Launch-Dateien und Konfigurationen
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp nav_msgs)

install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)

target_link_libraries(test_node Eigen3::Eigen)



add_executable(recorder_node src/recorder_node.cpp)
ament_target_dependencies(recorder_node rclcpp nav_msgs ament_index_cpp geometry_msgs sensor_msgs)

add_executable(joint_noise_node src/joint_noise_node.cpp)
ament_target_dependencies(joint_noise_node rclcpp sensor_msgs)

add_executable(noisy_odometry_node src/noisy_odometry_node.cpp)
ament_target_dependencies(noisy_odometry_node rclcpp sensor_msgs nav_msgs)

add_executable(kalman_filter_node src/kalman_filter_node.cpp)
ament_target_dependencies(kalman_filter_node rclcpp nav_msgs geometry_msgs)
target_include_directories(kalman_filter_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(extended_kalman_filter_node src/extended_kalman_filter_node.cpp)
ament_target_dependencies(extended_kalman_filter_node rclcpp nav_msgs sensor_msgs geometry_msgs)
target_include_directories(extended_kalman_filter_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(landmark_test_node src/landmark_test_node.cpp)
ament_target_dependencies(landmark_test_node rclcpp sensor_msgs geometry_msgs)
target_include_directories(landmark_test_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(ekf_landmark_node src/ekf_landmark.cpp)
ament_target_dependencies(ekf_landmark_node rclcpp sensor_msgs geometry_msgs nav_msgs)
target_include_directories(ekf_landmark_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  joint_noise_node
  noisy_odometry_node
  kalman_filter_node
  extended_kalman_filter_node
  landmark_test_node
  ekf_landmark_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS recorder_node DESTINATION lib/${PROJECT_NAME})




install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY trajectories DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
