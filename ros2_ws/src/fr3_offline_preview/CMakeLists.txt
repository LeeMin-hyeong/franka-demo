cmake_minimum_required(VERSION 3.16)
project(fr3_offline_preview)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

# 콘솔 예제와 같은 계산 코드를 컴파일한다. ROS2나 libfranka 의존성은 계산기에 없다.
set(GENERATOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../standalone/libfranka_demo")
add_executable(joint_state_preview src/joint_state_preview.cpp "${GENERATOR_DIR}/trajectory_generator.cpp")
target_include_directories(joint_state_preview PRIVATE "${GENERATOR_DIR}")
target_compile_features(joint_state_preview PRIVATE cxx_std_17)
target_compile_options(joint_state_preview PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(joint_state_preview rclcpp sensor_msgs urdf)
install(TARGETS joint_state_preview DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
