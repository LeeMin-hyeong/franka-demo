cmake_minimum_required(VERSION 3.16)
project(fr3_offline_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(fr3_trajectory trajectory_generator.cpp)
target_include_directories(fr3_trajectory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(fr3_trajectory PRIVATE -Wall -Wextra -Wpedantic)

add_executable(offline_demo offline_demo.cpp)
target_link_libraries(offline_demo PRIVATE fr3_trajectory)
target_compile_options(offline_demo PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  add_executable(trajectory_tests trajectory_tests.cpp)
  target_link_libraries(trajectory_tests PRIVATE fr3_trajectory)
  target_compile_options(trajectory_tests PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME quintic_trajectory COMMAND trajectory_tests)
  add_test(NAME offline_default COMMAND offline_demo)
  add_test(NAME offline_custom_period COMMAND offline_demo --joint 2 --delta -0.1 --duration 3.1 --rate 37)
  add_test(NAME reject_joint_limit COMMAND offline_demo --joint 4 --delta 10)
  set_tests_properties(reject_joint_limit PROPERTIES WILL_FAIL TRUE)
  add_test(NAME reject_short_duration COMMAND offline_demo --duration 0.01)
  set_tests_properties(reject_short_duration PROPERTIES WILL_FAIL TRUE)
endif()

# 기본 OFF: 실제 API를 컴파일하는 OBJECT만 만들며 실행 파일은 만들지 않는다.
# 객체 파일의 함수를 호출할 프로그램이 없으므로 빌드로 로봇에 연결되지 않는다.
option(COMPILE_REAL_API_EXAMPLE "Compile libfranka 0.20.4 API example without an executable" OFF)
if(COMPILE_REAL_API_EXAMPLE)
  find_package(Franka 0.20.4 EXACT REQUIRED)
  add_library(real_api_compile_check OBJECT real_control_example.cpp)
  target_link_libraries(real_api_compile_check PRIVATE fr3_trajectory Franka::Franka)
  target_compile_options(real_api_compile_check PRIVATE -Wall -Wextra -Wpedantic)
endif()
