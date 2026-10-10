/**
 * @file joint_state_preview.cpp
 * @brief C++ 5차 관절 궤적을 JointState로 전달하는 오프라인 RViz 재생 노드.
 * 라이브러리: rclcpp, sensor_msgs, urdf, 공용 trajectory_generator.
 * 입력: 공식 URDF, SRDF ready 초기값, joint(1~7), delta(rad), duration(s), rate(Hz).
 * 출력: /joint_states 위치(rad)와 속도(rad/s), 시작/목표/최종 위치 로그.
 * 흐름: 모델 제한 확인 -> 계산기 생성 -> 초기 자세 유지 -> 재생 -> 최종 자세 유지.
 * preview.launch.py가 robot_state_publisher와 RViz를 실행한다.
 * 이 파일은 libfranka와 링크하지 않는다. FCI 제어, 물리 시뮬레이션, 충돌 검사는 없다.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include "trajectory_generator.h"

namespace
{
/**
 * @brief 출력할 각도(rad)를 공식 이름과 함께 표시한다. 반환값과 예외 없음.
 * @param label START, GOAL, FINAL 중 상태 설명. q는 관절 순서 1~7의 각도.
 * 목표값과 최종 샘플을 구분하여 시각화가 어떤 값을 전달하는지 보여준다.
 */
void print_positions(const rclcpp::Logger & logger, const char * label,
  const fr3_demo::JointArray & q)
{
  const auto names = fr3_demo::joint_names();
  for (std::size_t i = 0; i < q.size(); ++i) {
    RCLCPP_INFO(logger, "%s %s = %.6f rad", label, names[i].c_str(), q[i]);
  }
}

/**
 * @brief 발행 전에 공식 URDF의 관절 순서, 위치, 속도 제한을 계산기와 비교한다.
 * @param description launch가 공식 Xacro에서 생성한 URDF 문자열.
 * @return 없음. 모델 파싱, 관절, 제한 불일치는 예외로 처리한다.
 * 고정 상수와 다른 모델을 설치했을 때 잘못된 제한으로 궤적을 만들지 않기 위함이다.
 */
void verify_model(const std::string & description)
{
  urdf::Model model;
  if (!model.initString(description)) {
    throw std::runtime_error("Official FR3 URDF could not be parsed.");
  }
  const auto names = fr3_demo::joint_names();
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto joint = model.getJoint(names[i]);
    if (!joint || joint->type != urdf::Joint::REVOLUTE || !joint->limits) {
      throw std::runtime_error("Official revolute joint with limits missing: " + names[i]);
    }
    if (std::abs(joint->limits->lower - fr3_demo::kPositionLower[i]) > 1e-5 ||
      std::abs(joint->limits->upper - fr3_demo::kPositionUpper[i]) > 1e-5 ||
      std::abs(joint->limits->velocity - fr3_demo::kVelocityLimit[i]) > 1e-5)
    {
      throw std::runtime_error("URDF limits differ from verified franka_description 2.8.1: " + names[i]);
    }
  }
}
}  // namespace

/**
 * @brief 타이머에서 계산기를 샘플링하여 ROS2 메시지를 전달한다.
 * Node는 파라미터, Publisher, Timer를 소유하는 통신 단위다. 이 노드는 상태를
 * 구독해 제어하는 폐루프 제어기가 아니라 계산 결과만 내보내는 단방향 시각화다.
 */
class JointStatePreview : public rclcpp::Node
{
public:
  /**
   * @brief 파라미터를 검증한 뒤 publisher와 타이머를 준비한다.
   * 입력은 ROS 파라미터. 반환값 없음. 모델/시간/제한 오류는 예외.
   * 궤적을 한 샘플도 발행하기 전에 전체 경로의 해석적 속도/가속도를 검사한다.
   */
  JointStatePreview() : Node("fr3_joint_state_preview")
  {
    const auto description = declare_parameter<std::string>("robot_description", "");
    verify_model(description);
    const auto positions = declare_parameter<std::vector<double>>("initial_positions",
      std::vector<double>(fr3_demo::kHomePosition.begin(), fr3_demo::kHomePosition.end()));
    if (positions.size() != 7) {
      throw std::invalid_argument("Official ready state must have seven joint positions.");
    }
    std::copy(positions.begin(), positions.end(), initial_.begin());
    // launch는 SRDF의 ready를 읽어 전달한다. standalone 예제와 시작값이 같은지도 확인한다.
    for (std::size_t i = 0; i < initial_.size(); ++i) {
      if (std::abs(initial_[i] - fr3_demo::kHomePosition[i]) > 1e-9) {
        throw std::invalid_argument("Official SRDF ready differs from the verified demo home.");
      }
    }
    const auto joint = declare_parameter<int>("joint", 4);
    const auto delta = declare_parameter<double>("delta", 0.1);
    const auto duration = declare_parameter<double>("duration", 3.0);
    const auto rate = declare_parameter<double>("rate", 100.0);
    start_delay_ = declare_parameter<double>("start_delay", 15.0);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 1000.0 ||
      !std::isfinite(start_delay_) || start_delay_ < 0.0 || start_delay_ > 60.0)
    {
      throw std::invalid_argument("rate=1..1000 Hz and start_delay=0..60 s required.");
    }
    goal_ = fr3_demo::single_joint_goal(initial_, joint, delta);
    trajectory_ = std::make_unique<fr3_demo::QuinticTrajectory>(initial_, goal_, duration);
    RCLCPP_INFO(get_logger(), "OFFLINE PREVIEW: no FCI connection, no real-time guarantee, no collision check.");
    RCLCPP_INFO(get_logger(), "Official URDF loaded; SRDF ready; %.1f Hz, duration %.3f s", rate, duration);
    RCLCPP_INFO(get_logger(), "Holding ready pose for %.1f s before replay so RViz can load.", start_delay_);
    print_positions(get_logger(), "START", initial_);
    print_positions(get_logger(), "GOAL", goal_);
    // /joint_states 발행자를 추가하기 전에 다른 데모가 실행 중인지 확인한다.
    // 발견까지 시간이 필요한 ROS discovery 특성 때문에 타이머에서도 다시 검사한다.
    if (count_publishers("/joint_states") != 0) {
      throw std::runtime_error("Another /joint_states publisher exists. Stop MoveIt or the other preview.");
    }
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    const auto names = fr3_demo::joint_names();
    message_.name.assign(names.begin(), names.end());
    message_.position.resize(7);
    message_.velocity.resize(7);
    // effort는 계산하지 않았으므로 비워 둔다. JointState에는 name과 동일한 순서의
    // position, velocity를 넣는다. robot_state_publisher는 position+URDF로 TF를 계산한다.
    start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate)), [this]() {publish_sample();});
  }

private:
  /**
   * @brief ROS executor가 매 주기 호출한다. 현재 시간의 궤적을 /joint_states로 보낸다.
   * 입력 인자 없음. 출력은 타임스탬프를 포함하는 메시지다. 중복 발행자는 예외.
   * steady_clock으로 경과 시간을 계산하므로 주기가 늦어져도 궤적을 느리게 늘이지
   * 않는다. 이 방법은 FCI의 실시간 1 kHz callback 주기 보장을 제공하지 않는다.
   */
  void publish_sample()
  {
    if (count_publishers("/joint_states") > 1) {
      throw std::runtime_error("Multiple /joint_states publishers detected. Offline preview stopped.");
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_).count() - start_delay_;
    const auto sample = trajectory_->sample(elapsed);
    std::copy(sample.position.begin(), sample.position.end(), message_.position.begin());
    std::copy(sample.velocity.begin(), sample.velocity.end(), message_.velocity.begin());
    message_.header.stamp = now();
    publisher_->publish(message_);
    if (elapsed >= trajectory_->duration() && !finished_) {
      finished_ = true;
      print_positions(get_logger(), "FINAL", sample.position);
      RCLCPP_INFO(get_logger(), "Preview COMPLETE. Holding final pose; Ctrl+C to stop. No FCI API called.");
    }
  }

  fr3_demo::JointArray initial_{};
  fr3_demo::JointArray goal_{};
  std::unique_ptr<fr3_demo::QuinticTrajectory> trajectory_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState message_;
  std::chrono::steady_clock::time_point start_;
  double start_delay_ = 15.0;
  bool finished_ = false;
};

/**
 * @brief ROS를 시작하고 주기 콜백을 처리한다. 종료 시 노드 자원은 자동 정리된다.
 * @param argc, argv ROS 표준 파라미터 인자. @return 정상 종료 0, 검증/콜백 실패 1.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<JointStatePreview>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("fr3_joint_state_preview"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
