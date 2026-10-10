/**
 * @file real_control_example.cpp
 * @brief libfranka 0.20.4의 실제 FCI API 관계를 보여 주는 컴파일 전용 학습 파일.
 *
 * 사용 라이브러리는 libfranka와 이 저장소의 표준 C++17 궤적 생성기다.
 * 입력은 로봇 주소, 목표 관절각(rad), 시간(s)이고 출력 예제는 JointPositions다.
 * Robot 연결 -> readOnce 상태 조회 -> control 콜백 -> MotionFinished 순서를 보인다.
 * main이 없으며 기본 빌드에도 포함하지 않는다. 선택 빌드에서도 OBJECT만 만든다.
 * 첫 줄의 무조건 예외는 누군가 이 함수를 별도로 호출해도 연결을 방지한다.
 * 실제 실행으로 바꾸려면 실물 검증, FCI 설정, 실시간 환경, 추가 제한 검사가 필요하다.
 * 이 함수는 Offline Preview에서 호출하거나 링크하지 않는다.
 */
#include "trajectory_generator.h"

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

/**
 * @brief 실제 API 문법을 컴파일러로 확인한다. 실행 가능 함수로 제공하지 않는다.
 * @param robot_address FCI IP 또는 호스트명. 이 예제는 연결하기 전에 항상 중단한다.
 * @param goal 목표 관절각(rad).
 * @param duration_s 이동 시간(s).
 * @throws std::logic_error 항상 발생하며 연결을 막는다.
 * 아래 비실행 코드는 API 학습과 타입 검사에만 사용한다. 하드웨어 안전성은 미검증이다.
 */
void real_control_api_structure(const std::string& robot_address,
                                const fr3_demo::JointArray& goal, double duration_s) {
  throw std::logic_error("compile-only API example: real robot connection disabled");

  try {
    // 생성자 자체가 네트워크 연결을 수행한다. URDF 로딩과 관계없는 실제 FCI API다.
    // 지역 객체가 범위를 벗어나면 소멸자가 연결 자원을 정리한다(RAII).
    franka::Robot robot(robot_address);
    const franka::RobotState initial_state = robot.readOnce();
    // q는 측정 관절각, q_d는 로봇 상태가 제공하는 desired(목표) 관절각이다.
    // 처리 전 원시 명령과 같다고 가정하지 않으며 새 위치 명령은 q_d와 연속이어야 한다.
    std::cout << "Measured joint1: " << initial_state.q[0] << " rad\n";
    const fr3_demo::QuinticTrajectory trajectory(initial_state.q_d, goal, duration_s);
    double elapsed_s = 0.0;
    bool first_callback = true;

    // Robot::control은 수신한 RobotState와 실제 이전 호출 간격 Duration을 전달한다.
    // 첫 호출의 period는 0이다. 오프라인처럼 고정 주기를 더하지 않고 이 시간을 쓴다.
    // 실제 1kHz 콜백에서는 출력/할당/대기를 하지 않아야 한다. 이 구조는 학습용이다.
    robot.control([&](const franka::RobotState& state,
                      franka::Duration period) -> franka::JointPositions {
      if (first_callback) {
        // readOnce 뒤 상태가 바뀌었으면 계산한 시작점이 더 이상 적합하지 않다.
        // 실제 구현에는 최신 상태 연속성, jerk, 위치 의존 제한 등 추가 검증이 필요하다.
        for (std::size_t i = 0; i < state.q_d.size(); ++i) {
          if (std::abs(state.q_d[i] - initial_state.q_d[i]) > 1e-6) {
            throw std::runtime_error("commanded start state changed before control");
          }
        }
        first_callback = false;
      }
      elapsed_s += period.toSec();
      const auto sample = trajectory.sample(elapsed_s);
      franka::JointPositions command(sample.position);
      if (elapsed_s >= trajectory.duration()) {
        // motion_finished 플래그를 설정한 마지막 명령을 반환하여 제어 루프를 끝낸다.
        return franka::MotionFinished(command);
      }
      return command;
    });
  } catch (const franka::Exception& error) {
    // NetworkException, ControlException 등은 franka::Exception에서 파생된다.
    // 예외로 범위를 나가도 Robot 소멸자가 정리한다. 실패를 성공으로 바꾸지 않는다.
    std::cerr << "libfranka error: " << error.what() << '\n';
    throw;
  }
}
