cmake_minimum_required(VERSION 3.16)
project(fr3_moveit_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
# Humble 바이너리 OMPL export의 Boost::random 대상을 먼저 준비한다.
find_package(Boost REQUIRED COMPONENTS random)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(moveit_demo src/moveit_demo.cpp)
target_compile_features(moveit_demo PRIVATE cxx_std_17)
target_compile_options(moveit_demo PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(moveit_demo rclcpp moveit_ros_planning_interface sensor_msgs)
install(TARGETS moveit_demo DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
# --symlink-install은 원본을 가리키므로 Python 진입점의 Git 실행 권한이 필요하다.
install(PROGRAMS scripts/moveit_demo.py examples/basic_sequence.py examples/read_state.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
