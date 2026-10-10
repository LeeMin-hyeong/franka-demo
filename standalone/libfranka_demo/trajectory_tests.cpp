/**
 * @file trajectory_tests.cpp
 * @brief 로봇 없이 5차 보간의 경계, 미분값, 제한 거부 동작을 검증한다.
 *
 * 표준 C++17과 trajectory_generator만 사용한다. 테스트 입력은 고정 자세/시간이고
 * 출력은 실패 원인과 프로세스 종료 코드다. CTest가 빌드 후 이 파일을 실행한다.
 * 해석적 중간값 및 유한 차분과 비교하여 구현을 그대로 복사한 테스트를 피한다.
 * ROS2/RViz/FCI 동작은 여기서 검증하지 않는다.
 */
#include "trajectory_generator.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

int failures = 0;

/** @brief 조건이 false이면 실패를 누적한다. Release 빌드에서도 검증을 유지한다. */
void check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

/** @brief 두 수치가 지정한 절대 오차 이내인지 검사한다. 값의 단위는 호출부와 같다. */
void check_near(double actual, double expected, double tolerance, const std::string& label) {
  check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, label);
}

/** @brief 작업이 invalid_argument를 던지는지 확인한다. 던지지 않으면 실패로 누적한다. */
template <typename Function>
void expect_invalid(Function function, const std::string& label) {
  try {
    function();
    check(false, label + " was accepted");
  } catch (const std::invalid_argument&) {
    // 잘못된 입력이 실행 전에 거부되는 것이 이 테스트의 기대 동작이다.
  }
}

/** @brief 시작/종료 경계와 다중 관절 보간의 중간값, 연속성을 검증한다. */
void test_shape_and_derivatives() {
  const auto start = fr3_demo::kHomePosition;
  auto goal = start;
  goal[0] += 0.2;
  goal[1] -= 0.15;
  goal[3] += 0.1;
  const double duration = 3.0;
  const fr3_demo::QuinticTrajectory trajectory(start, goal, duration);
  check(trajectory.duration() == duration, "duration accessor");

  for (const double t : {-10.0, 0.0, duration, duration + 10.0}) {
    const auto value = trajectory.sample(t);
    check(value.position == (t <= 0.0 ? start : goal), "exact endpoint positions");
    for (std::size_t i = 0; i < start.size(); ++i) {
      check(value.velocity[i] == 0.0, "stationary endpoint velocity");
      check(value.acceleration[i] == 0.0, "stationary endpoint acceleration");
    }
  }

  const auto middle = trajectory.sample(duration / 2.0);
  for (std::size_t i = 0; i < start.size(); ++i) {
    check_near(middle.position[i], (start[i] + goal[i]) / 2.0, 1e-12,
               "midpoint symmetry");
    check_near(middle.velocity[i], 1.875 * (goal[i] - start[i]) / duration,
               1e-12, "analytic peak velocity");
    check_near(middle.acceleration[i], 0.0, 1e-12, "midpoint zero acceleration");
  }

  // 위치의 중앙 차분으로 속도를, 속도의 중앙 차분으로 가속도를 별도로 확인한다.
  for (const double t : {0.4, 1.1, 2.7}) {
    constexpr double epsilon = 1e-5;
    const auto before = trajectory.sample(t - epsilon);
    const auto value = trajectory.sample(t);
    const auto after = trajectory.sample(t + epsilon);
    for (std::size_t i = 0; i < start.size(); ++i) {
      check_near(value.velocity[i], (after.position[i] - before.position[i]) /
                                      (2.0 * epsilon), 1e-9, "velocity finite difference");
      check_near(value.acceleration[i], (after.velocity[i] - before.velocity[i]) /
                                          (2.0 * epsilon), 1e-9, "acceleration finite difference");
    }
  }

  const double first_peak = duration * (3.0 - std::sqrt(3.0)) / 6.0;
  const auto acceleration_peak = trajectory.sample(first_peak);
  check_near(acceleration_peak.acceleration[0],
             10.0 / std::sqrt(3.0) * (goal[0] - start[0]) / duration / duration,
             1e-12, "analytic acceleration maximum");

  // 양수 시간의 극단값에서도 정지 궤적은 0*무한대로 NaN이 되면 안 된다.
  const fr3_demo::QuinticTrajectory stationary(start, start, 1e-300);
  const auto stationary_value = stationary.sample(0.5e-300);
  check(stationary_value.position == start, "tiny-duration stationary positions");
  for (std::size_t i = 0; i < start.size(); ++i) {
    check(stationary_value.velocity[i] == 0.0 && stationary_value.acceleration[i] == 0.0,
          "tiny-duration stationary derivatives finite");
  }

  for (int step = 0; step <= 1000; ++step) {
    const auto value = trajectory.sample(duration * static_cast<double>(step) / 1000.0);
    fr3_demo::validate_positions(value.position);
    for (std::size_t i = 0; i < start.size(); ++i) {
      check(std::abs(value.velocity[i]) <= fr3_demo::kVelocityLimit[i], "sample velocity bound");
      check(std::abs(value.acceleration[i]) <= fr3_demo::kAccelerationLimit[i],
            "sample acceleration bound");
      if (goal[i] == start[i]) {
        check(value.position[i] == start[i] && value.velocity[i] == 0.0 &&
                  value.acceleration[i] == 0.0, "unmoved joint remains stationary");
      }
    }
  }
}

/** @brief 인덱스, 비유한 값, 위치/속도/가속도 제한 및 시간 오류를 거부하는지 검사한다. */
void test_rejections() {
  const auto start = fr3_demo::kHomePosition;
  const auto goal = fr3_demo::single_joint_goal(start, 4, 0.1);
  check_near(goal[3], start[3] + 0.1, 1e-12, "human joint number maps to index 3");
  check(fr3_demo::joint_names()[3] == "fr3_joint4", "model joint name order");
  fr3_demo::validate_positions(fr3_demo::kPositionLower);
  fr3_demo::validate_positions(fr3_demo::kPositionUpper);

  expect_invalid([&] { fr3_demo::single_joint_goal(start, 0, 0.1); }, "joint zero");
  expect_invalid([&] { fr3_demo::single_joint_goal(start, 8, 0.1); }, "joint eight");
  expect_invalid([&] { fr3_demo::single_joint_goal(start, 4, 10.0); }, "joint position limit");
  expect_invalid([&] {
    fr3_demo::single_joint_goal(start, 4, std::numeric_limits<double>::quiet_NaN());
  }, "NaN delta");
  auto bad_position = start;
  bad_position[0] = std::numeric_limits<double>::infinity();
  expect_invalid([&] { fr3_demo::QuinticTrajectory(bad_position, goal, 3.0); }, "infinite start");
  bad_position[0] = std::numeric_limits<double>::quiet_NaN();
  expect_invalid([&] { fr3_demo::QuinticTrajectory(start, bad_position, 3.0); }, "NaN goal");
  bad_position = start;
  bad_position[5] = 0.0;
  expect_invalid([&] { fr3_demo::validate_positions(bad_position); }, "joint6 zero invalid");
  for (const double duration : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::quiet_NaN()}) {
    expect_invalid([&] { fr3_demo::QuinticTrajectory(start, goal, duration); }, "invalid duration");
  }
  const auto fast_goal = fr3_demo::single_joint_goal(start, 1, 2.0);
  expect_invalid([&] { fr3_demo::QuinticTrajectory(start, fast_goal, 1.0); }, "velocity excess");
  // 0.1rad/0.2s는 속도 최대 0.9375rad/s로 유효하지만 가속도 약14.43으로 초과한다.
  expect_invalid([&] { fr3_demo::QuinticTrajectory(start, goal, 0.2); }, "acceleration excess");
  const fr3_demo::QuinticTrajectory trajectory(start, goal, 3.0);
  expect_invalid([&] { trajectory.sample(std::numeric_limits<double>::quiet_NaN()); }, "NaN time");
  expect_invalid([&] { trajectory.sample(std::numeric_limits<double>::infinity()); }, "infinite time");
}

}  // namespace

/** @brief 모든 독립 검사를 수행한다. 전체 통과는 0, 검증 실패/예외는 1을 반환한다. */
int main() {
  try {
    test_shape_and_derivatives();
    test_rejections();
  } catch (const std::exception& error) {
    std::cerr << "Unexpected exception: " << error.what() << '\n';
    return 1;
  }
  if (failures != 0) {
    std::cerr << failures << " checks failed\n";
    return 1;
  }
  std::cout << "Quintic boundary, derivative, FR3 limit and rejection checks passed.\n";
  return 0;
}
