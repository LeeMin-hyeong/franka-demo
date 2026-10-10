/**
 * @file trajectory_generator.cpp
 * @brief 표준 C++17만으로 5차 관절 궤적과 제한 검사를 구현한다.
 * 입력/출력 단위와 공개 함수 계약은 trajectory_generator.h에 정의한다.
 * 목표 생성 -> 전체 궤적 제한 검사 -> 원하는 시간의 샘플 계산 순서로 쓴다.
 * ROS2, MoveIt2, libfranka, 네트워크와 무관하므로 로봇 없이 테스트할 수 있다.
 */
#include "trajectory_generator.h"

#include <cmath>
#include <stdexcept>

namespace fr3_demo {

std::array<std::string, 7> joint_names() {
  return {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
          "fr3_joint5", "fr3_joint6", "fr3_joint7"};
}

void validate_positions(const JointArray& positions) {
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!std::isfinite(positions[i]) || positions[i] < kPositionLower[i] ||
        positions[i] > kPositionUpper[i]) {
      throw std::invalid_argument("Joint " + std::to_string(i + 1) +
                                  " position outside FR3 model limits (rad)");
    }
  }
}

JointArray single_joint_goal(const JointArray& start, int joint_number, double delta_rad) {
  validate_positions(start);
  if (joint_number < 1 || joint_number > 7 || !std::isfinite(delta_rad)) {
    throw std::invalid_argument("joint must be 1..7 and delta must be finite (rad)");
  }
  JointArray goal = start;
  // CLI 번호 4는 배열 index 3이다. 검증 후 변환하므로 범위를 벗어나지 않는다.
  goal[static_cast<std::size_t>(joint_number - 1)] += delta_rad;
  validate_positions(goal);
  return goal;
}

QuinticTrajectory::QuinticTrajectory(const JointArray& start, const JointArray& goal,
                                   double duration_s)
    : start_(start), goal_(goal), duration_s_(duration_s) {
  validate_positions(start_);
  validate_positions(goal_);
  if (!std::isfinite(duration_s_) || duration_s_ <= 0.0) {
    throw std::invalid_argument("duration must be finite and positive (s)");
  }

  // s'(u)=30u^2(1-u)^2이므로 0<=u<=1에서 s는 단조 증가한다.
  // 따라서 양 끝의 위치가 유효하면 그 사이의 모든 위치도 제한 안에 있다.
  // 최대 |s'|는 u=1/2에서 15/8, 최대 |s''|는 (3+-sqrt(3))/6에서
  // 10/sqrt(3)이다. 시간 미분 시 각각 1/T, 1/T^2가 붙는다.
  // 샘플만 검사하면 샘플 사이의 초과를 놓칠 수 있어 이 최대값을 사용한다.
  for (std::size_t i = 0; i < start_.size(); ++i) {
    const double distance = std::abs(goal_[i] - start_[i]);
    const double peak_velocity = (15.0 / 8.0) * distance / duration_s_;
    const double peak_acceleration = (10.0 / std::sqrt(3.0)) * distance /
                                     duration_s_ / duration_s_;
    if (!std::isfinite(peak_velocity) || peak_velocity > kVelocityLimit[i]) {
      throw std::invalid_argument("Joint " + std::to_string(i + 1) +
                                  " peak velocity exceeds limit; increase duration (s)");
    }
    if (!std::isfinite(peak_acceleration) || peak_acceleration > kAccelerationLimit[i]) {
      throw std::invalid_argument("Joint " + std::to_string(i + 1) +
                                  " peak acceleration exceeds limit; increase duration (s)");
    }
  }
}

TrajectorySample QuinticTrajectory::sample(double elapsed_s) const {
  if (!std::isfinite(elapsed_s)) {
    throw std::invalid_argument("sample time must be finite (s)");
  }
  TrajectorySample result;
  if (elapsed_s <= 0.0) {
    result.position = start_;
    return result;
  }
  if (elapsed_s >= duration_s_) {
    result.position = goal_;
    return result;
  }

  // u=t/T는 시간을 [0,1]로 정규화한다. 같은 곡선을 이동 시간만 바꿔 재사용한다.
  // 선형 보간은 정지 상태와 연결할 때 속도가 갑자기 바뀐다. 5차 보간은 양 끝의
  // s'=s''=0 조건을 만족하므로 속도와 가속도를 0으로 연결할 수 있다.
  // 하지만 jerk와 위치 의존 속도 제한, 충돌, FCI 필터는 별도 고려 사항이다.
  const double u = elapsed_s / duration_s_;
  const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  const double ds = 30.0 * u * u * (1.0 - u) * (1.0 - u);
  const double dds = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u);
  for (std::size_t i = 0; i < start_.size(); ++i) {
    const double delta = goal_[i] - start_[i];
    result.position[i] = start_[i] + s * delta;
    // 1/T^2를 먼저 만들면 아주 작은 T에서 무한대가 될 수 있다.
    // 검증한 관절 이동량을 먼저 T로 나누어 정지 관절의 0*무한대도 피한다.
    result.velocity[i] = (delta / duration_s_) * ds;
    result.acceleration[i] = (delta / duration_s_ / duration_s_) * dds;
  }
  return result;
}

}  // namespace fr3_demo
