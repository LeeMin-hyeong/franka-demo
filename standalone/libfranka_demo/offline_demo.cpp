/**
 * @file offline_demo.cpp
 * @brief 표준 C++17 콘솔에서 관절 궤적을 생성하고 선택적으로 CSV로 저장한다.
 *
 * CLI 입력은 관절 번호(1..7), 상대 이동(rad), 시간(s), 샘플 주파수(Hz)이다.
 * trajectory_generator.cpp로 전체 궤적을 검증하고 지정한 시간 격자에서 계산한다.
 * 출력은 시작/종료 자세, 샘플 수, 선택적 CSV이다. 실제 시간 재생은 하지 않는다.
 * ROS2 또는 libfranka를 링크하지 않으며 FCI 연결도 시도하지 않는다.
 * RViz 재생은 별도 joint_state_preview.cpp가 같은 생성기를 사용하여 담당한다.
 */
#include "trajectory_generator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Options {
  int joint = 4;
  double delta = 0.1;
  double duration = 3.0;
  double rate = 100.0;
  std::string csv_path;
  bool help = false;
};

/**
 * @brief CLI 문자열 전체를 유한한 실수로 해석한다. 반환 단위는 각 옵션의 단위.
 * 시작 시 호출한다. 일부만 숫자이거나 NaN/무한대이면 invalid_argument를 던진다.
 */
double parse_number(const std::string& value) {
  std::size_t consumed = 0;
  const double number = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(number)) {
    throw std::invalid_argument("expected a finite number: " + value);
  }
  return number;
}

/**
 * @brief main 시작 시 CLI 옵션을 해석하고 Options를 반환한다.
 * 단위는 delta rad, duration s, rate Hz. 잘못된 옵션/값은 예외로 거부한다.
 * 한 번의 미리보기로 지나치게 많은 샘플을 만들지 않도록 시간/주파수를 제한한다.
 */
Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help" || key == "-h") {
      options.help = true;
      continue;
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + key);
    }
    const std::string value = argv[++i];
    if (key == "--joint") {
      const double joint = parse_number(value);
      if (joint < 1.0 || joint > 7.0 || std::floor(joint) != joint) {
        throw std::invalid_argument("joint must be an integer from 1 to 7");
      }
      options.joint = static_cast<int>(joint);
    } else if (key == "--delta") {
      options.delta = parse_number(value);
    } else if (key == "--duration") {
      options.duration = parse_number(value);
    } else if (key == "--rate") {
      options.rate = parse_number(value);
    } else if (key == "--csv") {
      options.csv_path = value;
    } else {
      throw std::invalid_argument("unknown option: " + key);
    }
  }
  if (options.duration <= 0.0 || options.duration > 3600.0) {
    throw std::invalid_argument("duration must be in (0, 3600] s");
  }
  if (options.rate < 1.0 || options.rate > 1000.0) {
    throw std::invalid_argument("rate must be in [1, 1000] Hz (offline sampling only)");
  }
  return options;
}

/** @brief 입력 자세(rad)를 7개 관절 이름과 함께 표시한다. 시작/종료 때 호출한다. */
void print_positions(const char* label, const fr3_demo::JointArray& position) {
  const auto names = fr3_demo::joint_names();
  std::cout << label << " (rad):\n";
  for (std::size_t i = 0; i < position.size(); ++i) {
    std::cout << "  " << names[i] << " = " << position[i] << '\n';
  }
}

}  // namespace

/**
 * @brief 옵션 해석 -> 목표 검증 -> 샘플 생성 -> 최종 상태 출력 순서로 실행한다.
 * 성공/도움말은 0, 입력 오류 또는 CSV 저장 실패는 1을 반환한다.
 * 예외는 여기서 메시지로 바꾸므로 잘못된 목표로 부분 실행하지 않는다.
 */
int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    if (options.help) {
      std::cout << "Usage: offline_demo [--joint 1..7] [--delta rad] [--duration s]\n"
                   "                    [--rate Hz] [--csv path]\n"
                   "Defaults: joint=4 delta=0.1 duration=3 rate=100\n"
                   "Calculates samples only. No ROS2, FCI, or robot connection.\n";
      return 0;
    }
    const auto goal = fr3_demo::single_joint_goal(fr3_demo::kHomePosition,
                                                 options.joint, options.delta);
    const fr3_demo::QuinticTrajectory trajectory(fr3_demo::kHomePosition, goal,
                                                options.duration);
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "OFFLINE ONLY: no libfranka FCI or robot connection.\n"
              << "Joint " << options.joint << ", delta=" << options.delta
              << " rad, duration=" << options.duration << " s, rate=" << options.rate
              << " Hz (sample grid, no realtime guarantee)\n";
    print_positions("START / ready", trajectory.sample(0.0).position);
    print_positions("GOAL", goal);

    std::ofstream csv;
    if (!options.csv_path.empty()) {
      // exceptions()로 디스크/쓰기 실패도 잡는다. 잘못된 CSV를 성공으로 표시하지 않는다.
      csv.exceptions(std::ios::failbit | std::ios::badbit);
      csv.open(options.csv_path);
      csv << std::setprecision(12) << "time_s";
      for (const auto& name : fr3_demo::joint_names()) {
        csv << ',' << name << "_rad," << name << "_rad_s," << name << "_rad_s2";
      }
      csv << '\n';
    }

    // ceil을 사용하여 T가 샘플 주기의 정수배가 아니어도 마지막에 정확한 T를 쓴다.
    // 이 루프는 수학적 시간 격자를 계산한다. sleep이나 실제 1kHz 제어가 아니다.
    const auto steps = static_cast<std::size_t>(std::ceil(options.duration * options.rate));
    for (std::size_t step = 0; step <= steps; ++step) {
      const double t = std::min(static_cast<double>(step) / options.rate, options.duration);
      const auto sample = trajectory.sample(t);
      if (csv.is_open()) {
        csv << t;
        for (std::size_t i = 0; i < sample.position.size(); ++i) {
          csv << ',' << sample.position[i] << ',' << sample.velocity[i]
              << ',' << sample.acceleration[i];
        }
        csv << '\n';
      }
    }
    if (csv.is_open()) {
      csv.close();
      std::cout << "CSV saved: " << options.csv_path << '\n';
    }
    print_positions("FINAL", trajectory.sample(options.duration).position);
    std::cout << "Generated " << steps + 1 << " samples. Position/velocity/acceleration checks passed.\n"
                 "Collision, jerk, position-dependent FCI velocity, and hardware execution are not tested.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OFFLINE ERROR: " << error.what() << '\n';
    return 1;
  }
}
