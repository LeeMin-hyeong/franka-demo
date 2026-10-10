/**
 * @file moveit_demo.cpp
 * @brief 공식 FR3 팔 관절 또는 가상 그리퍼 폭을 계획하고 Fake Hardware에서 실행한다.
 * 라이브러리: rclcpp, MoveIt2 MoveGroupInterface, sensor_msgs.
 * 입력: joint(1~7), delta(rad), duration(s), home, wait_timeout(s), gripper_width(m).
 * gripper_width=-1은 팔 모드, 0 이상은 두 손가락 사이의 전체 개방 폭이다.
 * 출력: 계획 결과, 실행 결과, 실제 /joint_states에서 받은 최종 위치.
 * 흐름: 가상 모델 확인 -> 최신 상태 수신 -> 목표 제한 검사 -> 계획 -> 실행 -> 검증.
 * fake_moveit.launch.py가 move_group과 ros2_control을 실행하고 이 파일은 클라이언트다.
 * 자체 IK나 궤적 생성기를 사용하지 않으며 실물 하드웨어 모델은 거부한다.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace
{
/** 실제 메시지를 소유한다. executor 콜백과 메인 스레드 사이 접근을 mutex로 보호한다. */
struct JointSnapshot
{
  std::mutex mutex;
  std::map<std::string, double> positions;
  std::size_t sequence = 0;
};

/**
 * @brief marker 이후 새 JointState 위치를 기다린다. 모델의 기본값을 수신 상태로 오인하지 않는다.
 * @param snapshot 구독 콜백이 채우는 위치(rad 또는 m), marker 이미 확인한 메시지 번호.
 * @param timeout 최대 대기 시간(s). 필요한 관절은 구독 콜백에서 미리 검사한다.
 * @return 이름별 실제 위치. 시간 초과 또는 ROS 종료는 예외로 보고한다.
 */
std::map<std::string, double> fresh_positions(const std::shared_ptr<JointSnapshot> & snapshot,
  std::size_t marker, double timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(snapshot->mutex);
      if (snapshot->sequence > marker) {
        return snapshot->positions;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("Current Robot State not received for required joints. Motion cancelled.");
}

/**
 * @brief 실행 중인 move_group의 URDF와 SRDF를 읽어 클라이언트 모델을 맞춘다.
 * @param node 결과를 저장할 ROS 노드. 별도 executor가 이미 spin해야 한다.
 * @param timeout 응답을 기다릴 최대 시간(s). 실패나 실물 모델이면 예외를 던진다.
 * @return 없음. Node의 robot_description 파라미터 두 개가 설정된다.
 * MoveIt 서버의 모델과 다른 모델로 목표를 만드는 실수를 방지하기 위한 단계다.
 */
void load_fake_model(const rclcpp::Node::SharedPtr & node, double timeout)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  const auto wait = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout));
  if (!client->wait_for_service(wait)) {
    throw std::runtime_error("/move_group parameter service timeout. Start Fake Hardware first.");
  }
  auto result = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (result.wait_for(wait) != std::future_status::ready) {
    throw std::runtime_error("URDF/SRDF parameter response timeout.");
  }
  const auto parameters = result.get();
  if (parameters.size() != 2 || parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
    parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    throw std::runtime_error("move_group did not provide URDF and SRDF strings.");
  }
  const auto & urdf = parameters[0].as_string();
  if (urdf.find("mock_components/GenericSystem") == std::string::npos ||
    urdf.find("franka_hardware/FrankaHardwareInterface") != std::string::npos)
  {
    throw std::runtime_error("This demo requires GenericSystem Fake Hardware; real hardware is refused.");
  }
  for (const auto & parameter : parameters) {
    if (!node->has_parameter(parameter.get_name())) {
      node->declare_parameter<std::string>(parameter.get_name(), parameter.as_string());
    } else {
      node->set_parameter(parameter);
    }
  }
}

/**
 * @brief 관절 이름과 위치를 단위와 함께 한 줄씩 출력한다.
 * @param label 상태 이름, names 모델 관절 순서, positions 같은 순서의 위치, unit rad 또는 m.
 * @return 없음. 상태 수신 후 또는 목표 생성 후 호출한다.
 */
void print_positions(const rclcpp::Logger & logger, const std::string & label,
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const std::string & unit)
{
  for (std::size_t i = 0; i < names.size(); ++i) {
    RCLCPP_INFO(logger, "%s %s = %.6f %s", label.c_str(), names[i].c_str(),
      positions.at(i), unit.c_str());
  }
}

/**
 * @brief 계획된 경로를 바꾸지 않고 이동 시간이 최소 requested(s)가 되도록 느리게 만든다.
 * @param plan MoveIt이 생성한 시간, 위치, 속도, 가속도를 가진 계획.
 * @param requested 사용자가 지정한 최소 이동 시간(s).
 * @return 실행될 전체 시간(s). 빈 경로나 유효하지 않은 시간이면 예외.
 * MoveIt의 시간 매개화 결과가 더 느리면 그대로 둔다. 시간은 scale배, 속도는
 * 1/scale배, 가속도는 1/scale²배이므로 기존 제한을 넘기도록 가속하지 않는다.
 */
double slow_plan(moveit::planning_interface::MoveGroupInterface::Plan & plan, double requested)
{
  auto & points = plan.trajectory_.joint_trajectory.points;
  if (points.empty()) {
    throw std::runtime_error("Planning returned an empty joint trajectory.");
  }
  const double original = rclcpp::Duration(points.back().time_from_start).seconds();
  if (!std::isfinite(original) || original <= 0.0) {
    throw std::runtime_error("Planning returned a trajectory without a positive duration.");
  }
  const double scale = std::max(1.0, requested / original);
  for (auto & point : points) {
    point.time_from_start = rclcpp::Duration::from_seconds(
      rclcpp::Duration(point.time_from_start).seconds() * scale);
    for (auto & velocity : point.velocities) {
      velocity /= scale;
    }
    for (auto & acceleration : point.accelerations) {
      acceleration /= scale * scale;
    }
  }
  return original * scale;
}

/**
 * @brief 팔 상대 이동/ready 복귀 또는 그리퍼 폭 변경을 한 번 계획하고 실행한다.
 * @param node 파라미터와 로그를 소유한 ROS 노드. executor가 별도 스레드에서 spin한다.
 * @return 성공 시 0. 상태, 모델, 제한, 계획, 실행, 최종 확인 실패는 예외로 전달한다.
 */
int run_demo(const rclcpp::Node::SharedPtr & node)
{
  const auto logger = node->get_logger();
  const auto joint = node->declare_parameter<int>("joint", 4);
  const auto delta = node->declare_parameter<double>("delta", 0.1);
  const auto duration = node->declare_parameter<double>("duration", 3.0);
  const auto home = node->declare_parameter<bool>("home", false);
  const auto timeout = node->declare_parameter<double>("wait_timeout", 30.0);
  const auto width = node->declare_parameter<double>("gripper_width", -1.0);
  const bool gripper_mode = width >= 0.0;
  if (joint < 1 || joint > 7 || !std::isfinite(delta) ||
    !std::isfinite(duration) || duration <= 0.0 || duration > 3600.0 ||
    !std::isfinite(timeout) || timeout <= 0.0 || timeout > 300.0 ||
    !std::isfinite(width) || (width < 0.0 && width != -1.0) || (gripper_mode && home))
  {
    throw std::invalid_argument("joint=1..7, finite delta, 0<duration<=3600, 0<wait_timeout<=300, gripper_width=-1 or >=0 required; home is arm-only.");
  }
  load_fake_model(node, timeout);
  const std::string group_name = gripper_mode ? "fr3_hand" : "fr3_arm";
  const std::string unit = gripper_mode ? "m" : "rad";
  const double tolerance = gripper_mode ? 0.0002 : 0.002;
  RCLCPP_INFO(logger, "C++ RViz demo: MoveGroupInterface, Fake Hardware only. No FCI connection. Planning group: %s",
    group_name.c_str());

  // Node는 통신의 단위다. executor가 action 응답과 JointState 콜백을 처리해야
  // getCurrentState(), plan(), execute()를 기다리는 메인 스레드가 진행할 수 있다.
  moveit::planning_interface::MoveGroupInterface move_group(
    node, group_name, std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(timeout));
  const auto model = move_group.getRobotModel();
  const auto * group = model->getJointModelGroup(group_name);
  if (!group) {
    throw std::runtime_error("Official planning group was not loaded: " + group_name);
  }
  // 손의 두 번째 관절은 mimic이다. 계획 목표와 컨트롤러 경로에는 독립 관절만 보낸다.
  const auto names = group->getActiveJointModelNames();
  std::vector<std::string> arm_names;
  for (int i = 1; i <= 7; ++i) {
    arm_names.push_back("fr3_joint" + std::to_string(i));
  }
  if ((!gripper_mode && names != arm_names) ||
    (gripper_mode && names != std::vector<std::string>{"fr3_finger_joint1"}))
  {
    throw std::runtime_error("Unexpected active joints in the official FR3 group.");
  }
  std::vector<std::string> required_names = arm_names;
  if (gripper_mode) {
    const auto * finger1 = model->getJointModel("fr3_finger_joint1");
    const auto * finger2 = model->getJointModel("fr3_finger_joint2");
    if (!finger1 || !finger2 || finger1->getType() != moveit::core::JointModel::PRISMATIC ||
      finger2->getType() != moveit::core::JointModel::PRISMATIC ||
      finger2->getMimic() != finger1 || finger2->getMimicFactor() != 1.0 ||
      finger2->getMimicOffset() != 0.0)
    {
      throw std::runtime_error("Official Franka Hand prismatic/mimic model is missing.");
    }
    required_names.push_back("fr3_finger_joint1");
    required_names.push_back("fr3_finger_joint2");
    // 두 관절 모두의 URDF 한계를 사용한다. 실제 libfranka Gripper API는 호출하지 않는다.
    for (const auto & name : std::vector<std::string>{"fr3_finger_joint1", "fr3_finger_joint2"}) {
      const auto & bounds = model->getVariableBounds(name);
      if (width / 2.0 < bounds.min_position_ || width / 2.0 > bounds.max_position_) {
        throw std::invalid_argument("Gripper width exceeds official joint bounds (0..0.08 m).");
      }
    }
  }

  // 별도 구독은 모든 관절이 실제 메시지로 전달되었는지 확인한다.
  // URDF의 기본값을 Current State로 오인하지 않는다. MoveIt도 자체 상태 모니터를 쓴다.
  auto snapshot = std::make_shared<JointSnapshot>();
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SystemDefaultsQoS(),
    [required_names, snapshot](sensor_msgs::msg::JointState::ConstSharedPtr message) {
      if (message->name.size() != message->position.size()) {
        return;
      }
      const bool complete = std::all_of(required_names.begin(), required_names.end(), [&](const std::string & name) {
          const auto found = std::find(message->name.begin(), message->name.end(), name);
          return found != message->name.end() &&
                 std::isfinite(message->position[std::distance(message->name.begin(), found)]);
        });
      if (!complete) {
        return;
      }
      std::map<std::string, double> positions;
      for (std::size_t i = 0; i < message->name.size(); ++i) {
        if (!positions.emplace(message->name[i], message->position[i]).second) {
          return;
        }
      }
      std::lock_guard<std::mutex> lock(snapshot->mutex);
      snapshot->positions = std::move(positions);
      ++snapshot->sequence;
    });
  move_group.startStateMonitor(timeout);
  auto current = move_group.getCurrentState(timeout);
  const auto initial_message = fresh_positions(snapshot, 0, timeout);
  if (!current) {
    throw std::runtime_error("MoveIt Current Robot State was not received. Motion cancelled.");
  }
  if (node->count_publishers("/joint_states") != 1) {
    throw std::runtime_error("Exactly one /joint_states publisher required; stop the other demo.");
  }
  for (const auto & name : required_names) {
    const auto & bounds = model->getVariableBounds(name);
    if (bounds.position_bounded_ && (initial_message.at(name) < bounds.min_position_ ||
      initial_message.at(name) > bounds.max_position_))
    {
      throw std::runtime_error("Received current state exceeds official joint bounds: " + name);
    }
  }
  if (!current->satisfiesBounds(group)) {
    throw std::runtime_error("Current state is outside official URDF joint bounds.");
  }
  std::vector<double> initial;
  for (const auto & name : names) {
    initial.push_back(initial_message.at(name));
  }
  // 원시 JointState의 양쪽 손가락도 확인한다. RobotState가 계산한 mimic 값만 검사하면
  // 발행되지 않은 두 번째 손가락 상태를 실제 수신 상태로 오인할 수 있다.
  auto report = [&](const std::string & label, const std::vector<double> & positions, double finger2) {
      print_positions(logger, label, names, positions, unit);
      if (gripper_mode) {
        RCLCPP_INFO(logger, "%s fr3_finger_joint2 = %.6f m", label.c_str(), finger2);
        RCLCPP_INFO(logger, "%s gripper_width = %.6f m", label.c_str(), positions[0] + finger2);
      }
    };
  if (gripper_mode && std::abs(initial[0] - initial_message.at("fr3_finger_joint2")) > tolerance) {
    throw std::runtime_error("Received finger states do not satisfy the official mimic relation.");
  }
  report("START", initial, gripper_mode ? initial_message.at("fr3_finger_joint2") : 0.0);
  std::vector<double> goal = initial;
  if (gripper_mode) {
    // 전체 폭은 양쪽 손가락 위치의 합이다. 한 독립 관절에는 전체 폭의 절반을 보낸다.
    goal[0] = width / 2.0;
  } else if (home) {
    const auto ready = move_group.getNamedTargetValues("ready");
    if (ready.size() != 7) {
      throw std::runtime_error("Official SRDF ready named state is missing or incomplete.");
    }
    for (std::size_t i = 0; i < names.size(); ++i) {
      goal[i] = ready.at(names[i]);
    }
    RCLCPP_INFO(logger, "HOME: official SRDF named state ready");
  } else {
    // CLI의 Joint 4는 배열 index 3이다. 받은 현재 상태에서 상대 목표를 계산한다.
    goal.at(static_cast<std::size_t>(joint - 1)) += delta;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto & bounds = model->getVariableBounds(names[i]);
    RCLCPP_INFO(logger, "LIMIT %s [%.4f, %.4f] %s", names[i].c_str(),
      bounds.min_position_, bounds.max_position_, unit.c_str());
    if (!std::isfinite(goal[i]) || (bounds.position_bounded_ &&
      (goal[i] < bounds.min_position_ || goal[i] > bounds.max_position_)))
    {
      throw std::invalid_argument("Target exceeds official joint bounds: " + names[i]);
    }
  }
  report("GOAL", goal, gripper_mode ? goal[0] : 0.0);
  const bool unchanged = std::equal(initial.begin(), initial.end(), goal.begin(),
    [](double a, double b) {return std::abs(a - b) < 1e-9;});
  if (unchanged) {
    RCLCPP_INFO(logger, "Already at target. No trajectory needed.");
    report("FINAL", initial, gripper_mode ? initial_message.at("fr3_finger_joint2") : 0.0);
    return 0;
  }

  // 낮은 속도 배율로 가상 이동을 관찰하기 쉽게 한다. 충돌 검사와 경로 탐색은
  // 공식 OMPL/MoveIt 서버가 담당하며 여기에서 자체 IK나 충돌 알고리즘을 만들지 않는다.
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPlanningTime(10.0);
  move_group.setGoalJointTolerance(gripper_mode ? 1e-5 : 1e-4);
  move_group.setStartState(*current);
  std::map<std::string, double> joint_goal;
  for (std::size_t i = 0; i < names.size(); ++i) {
    joint_goal[names[i]] = goal[i];
  }
  if (!move_group.setJointValueTarget(joint_goal)) {
    throw std::invalid_argument("MoveIt rejected the joint goal.");
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planned = move_group.plan(plan);
  if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Planning failed, MoveIt error=" + std::to_string(planned.val));
  }
  if (plan.trajectory_.joint_trajectory.joint_names != names) {
    throw std::runtime_error("Planned trajectory does not match the active group joints.");
  }
  const double execution_duration = slow_plan(plan, duration);
  RCLCPP_INFO(logger, "Planning SUCCESS. Requested minimum %.3f s; execution %.3f s",
    duration, execution_duration);

  // plan()은 계산만 하고 execute()가 컨트롤러 action에 궤적을 전달한다.
  // ros2_control GenericSystem이 위치 명령을 상태에 반영하고 RViz는 그 결과를 표시한다.
  if (node->count_publishers("/joint_states") != 1) {
    throw std::runtime_error("Another JointState publisher appeared. Execution cancelled.");
  }
  const auto executed = move_group.execute(plan);
  if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Execution failed, MoveIt error=" + std::to_string(executed.val));
  }
  // 명령한 목표를 최종 상태라고 출력하지 않는다. 실행 후 새 JointState를 기다린다.
  std::size_t marker;
  {
    std::lock_guard<std::mutex> lock(snapshot->mutex);
    marker = snapshot->sequence;
  }
  const auto final_message = fresh_positions(snapshot, marker, timeout);
  std::vector<double> final_positions;
  for (const auto & name : names) {
    final_positions.push_back(final_message.at(name));
  }
  report("FINAL", final_positions, gripper_mode ? final_message.at("fr3_finger_joint2") : 0.0);
  for (std::size_t i = 0; i < goal.size(); ++i) {
    if (std::abs(goal[i] - final_positions[i]) > tolerance) {
      throw std::runtime_error("Final joint state differs from target: " + names[i]);
    }
  }
  if (gripper_mode && (std::abs(final_message.at("fr3_finger_joint2") - goal[0]) > tolerance ||
    std::abs(final_positions[0] - final_message.at("fr3_finger_joint2")) > tolerance ||
    std::abs(final_positions[0] + final_message.at("fr3_finger_joint2") - width) > tolerance))
  {
    throw std::runtime_error("Final gripper width or mimic finger differs from target.");
  }
  RCLCPP_INFO(logger, "Execution SUCCESS. Final JointState matches goal (tolerance %.4f %s).",
    tolerance, unit.c_str());
  return 0;
}
}  // namespace

/**
 * @brief ROS를 초기화하고 통신 스레드와 데모 실행을 종료 시 정리한다.
 * @param argc, argv ROS 표준 --ros-args 파라미터 인자.
 * @return 성공 0, 입력/상태/계획/실행 오류 1. 실제 Robot 연결 코드는 없다.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fr3_moveit_demo");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int result = 1;
  try {
    result = run_demo(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
