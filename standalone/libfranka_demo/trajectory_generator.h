/**
 * @file trajectory_generator.h
 * @brief ROS2나 libfranka 없이 사용하는 FR3 오프라인 관절 궤적 계산 인터페이스.
 *
 * 입력은 시작/목표 관절각(rad), 이동 시간(s), 조회 시간(s)이다.
 * 출력은 7개 관절의 위치(rad), 속도(rad/s), 가속도(rad/s^2)이다.
 * 생성 시 제한을 검사하고 sample()로 5차 보간 값을 계산한다.
 * offline_demo.cpp와 ROS2 joint_state_preview.cpp가 이 코드를 함께 사용한다.
 * 실제 FCI 통신, 충돌 검사, 역기구학, 실시간 스케줄링은 수행하지 않는다.
 */
#pragma once

#include <array>
#include <string>

namespace fr3_demo {

using JointArray = std::array<double, 7>;

// franka_description 2.8.1 robots/fr3/joint_limits.yaml의 위치/속도 제한.
// URDF 모델에 맞추는 고정 제한이며 실제 FCI의 위치 의존 속도 제한과 다르다.
inline constexpr JointArray kPositionLower{
    -2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508};
inline constexpr JointArray kPositionUpper{
    2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
inline constexpr JointArray kVelocityLimit{2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};

// libfranka 0.20.4 include/franka/rate_limiting.h의 kMaxJointAcceleration.
// 10 rad/s^2에서 수치 오차 여유 kLimitEps = 0.001을 뺀 값이다.
inline constexpr JointArray kAccelerationLimit{9.999, 9.999, 9.999, 9.999,
                                              9.999, 9.999, 9.999};

// franka_description 2.8.1 robots/common/group_definition.xacro의 named state ready.
// 공식 franka_ros2 Humble Fake Hardware 초기값 및 libfranka 공식 예제와도 일치.
// 전부 0인 자세는 joint4와 joint6의 제한을 벗어나므로 사용하지 않는다.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr JointArray kHomePosition{0.0, -kPi / 4.0, 0.0, -3.0 * kPi / 4.0,
                                         0.0, kPi / 2.0, kPi / 4.0};

/** @brief 기본 공식 모델의 관절 이름 7개를 순서대로 반환한다. 예외 없음. */
std::array<std::string, 7> joint_names();

/**
 * @brief 궤적 생성 전에 관절 위치(rad)를 검사한다. 반환값 없음.
 * @throws std::invalid_argument 값이 유한하지 않거나 모델 위치 제한 밖일 때.
 */
void validate_positions(const JointArray& positions);

/**
 * @brief 시작 자세에서 관절 하나를 상대 이동한 목표 자세를 반환한다.
 * @param start 시작 관절각(rad), 모두 모델 제한 안이어야 한다.
 * @param joint_number 사람이 사용하는 관절 번호, 1부터 7까지.
 * @param delta_rad 해당 관절의 상대 이동량(rad).
 * @throws std::invalid_argument 번호, 시작값, 이동량, 목표값이 유효하지 않을 때.
 * 사용자가 명령을 입력한 직후 호출하며, 이동 전에 목표를 거부할 수 있다.
 */
JointArray single_joint_goal(const JointArray& start, int joint_number, double delta_rad);

struct TrajectorySample {
  JointArray position{};
  JointArray velocity{};
  JointArray acceleration{};
};

/**
 * @brief 정지 상태에서 출발하여 정지 상태로 끝나는 5차 관절 위치 보간.
 *
 * ROS2 노드와 콘솔 예제에 필요한 계산만 담는다. 입력 검증은 생성 시 한 번
 * 수행하며, 샘플링 주기와 별개로 해석적인 최대 속도/가속도를 검사한다.
 * 이 검사만으로 실제 로봇 안전성이나 FCI 명령 적합성이 보장되지는 않는다.
 */
class QuinticTrajectory {
 public:
  /**
   * @param start 시작 관절각(rad).
   * @param goal 목표 관절각(rad).
   * @param duration_s 이동 시간(s), 유한한 양수.
   * @throws std::invalid_argument 위치/시간이 유효하지 않거나 속도/가속도 초과.
   * 미리보기 시작 전에 생성하므로 잘못된 궤적을 일부 재생하지 않는다.
   */
  QuinticTrajectory(const JointArray& start, const JointArray& goal, double duration_s);

  /**
   * @brief elapsed_s(s) 시점의 위치/속도/가속도를 반환한다.
   * 0 이전은 정확한 시작값, T 이후는 정확한 목표값과 0 미분값을 반환한다.
   * 타이머 또는 오프라인 샘플 루프에서 호출한다. 메모리 할당은 하지 않는다.
   * @throws std::invalid_argument 조회 시간이 NaN 또는 무한대일 때.
   */
  TrajectorySample sample(double elapsed_s) const;

  /** @brief 검증된 이동 시간(s)을 반환한다. 예외 없음. */
  double duration() const noexcept { return duration_s_; }

 private:
  JointArray start_;
  JointArray goal_;
  double duration_s_;
};

}  // namespace fr3_demo
