// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"



namespace fyt::auto_aim {
/**
 * @brief 构造函数，初始化弹道解算器
 * @param n ROS2节点的弱指针引用，用于参数声明和获取
 */
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  // 获取解算器相关参数
  // 射击范围宽度和高度（单位：米）
  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 3.0);
  // 云台最大跟踪角速度（单位：rad/s）
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  // 预测延迟时间（单位：秒）
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  // 控制器延迟时间（单位：秒）
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  // 侧装甲相对于中心的角度（单位：度）
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  // 最小切换角速度阈值（单位：rad/s）
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);
  // Coming/Leaving 非对称角度（单位：度）
  coming_angle_ = node->declare_parameter("solver.coming_angle", 55.0);
  leaving_angle_ = node->declare_parameter("solver.leaving_angle", 20.0);
  // 距离分级开火决策参数
  fire_margin_ = node->declare_parameter("solver.fire_margin", 0.8);
  double min_tol_deg = node->declare_parameter("solver.min_fire_tolerance", 1.5);
  double max_tol_deg = node->declare_parameter("solver.max_fire_tolerance", 4.0);
  min_fire_tolerance_rad_ = min_tol_deg * M_PI / 180.0;
  max_fire_tolerance_rad_ = max_tol_deg * M_PI / 180.0;

  // 瞄准偏移量（与 launch rpy 解耦，不影响 yaw 优化）
  yaw_offset_ = node->declare_parameter("solver.yaw_offset", 0.0);
  pitch_offset_ = node->declare_parameter("solver.pitch_offset", 0.0);

  // 初始化弹道补偿器
  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    coming_angle_ = node->get_parameter("solver.coming_angle").as_double();
    leaving_angle_ = node->get_parameter("solver.leaving_angle").as_double();
    fire_margin_ = node->get_parameter("solver.fire_margin").as_double();
    yaw_offset_ = node->get_parameter("solver.yaw_offset").as_double();
    pitch_offset_ = node->get_parameter("solver.pitch_offset").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  std::array<double, 3> rpy;
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
    rpy[0] = -rpy[1];
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Iterative fly-time prediction for accurate target position
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d target_velocity(target.velocity.x, target.velocity.y, target.velocity.z);
  double target_yaw = target.yaw;
  double processing_delay = (current_time - rclcpp::Time(target.header.stamp)).seconds() + prediction_delay_;
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);

  // Iterative convergence loop: target moves during flight, changing the required fly time
  int saved_lock_id = lock_id_;
  for (int iter = 0; iter < 10; ++iter) {
    double dt_total = processing_delay + flying_time;
    Eigen::Vector3d predicted_pos = Eigen::Vector3d(target.position.x, target.position.y, target.position.z)
                                    + dt_total * target_velocity;
    double predicted_yaw = target.yaw + dt_total * target.v_yaw;
    auto predicted_armors = getArmorPositions(
      predicted_pos, predicted_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
    // Use closest armor for fly-time estimation without altering lock state
    double min_dist = 1e9;
    int approx_idx = 0;
    for (size_t i = 0; i < predicted_armors.size(); i++) {
      double d = predicted_armors[i].head<2>().norm();
      if (d < min_dist) { min_dist = d; approx_idx = static_cast<int>(i); }
    }
    double new_fly_time = trajectory_compensator_->getFlyingTime(predicted_armors.at(approx_idx));
    if (std::abs(new_fly_time - flying_time) < 0.001) {
      flying_time = new_fly_time;
      break;
    }
    flying_time = new_fly_time;
  }
  lock_id_ = saved_lock_id;

  double dt = processing_delay + flying_time;
  target_position += dt * target_velocity;
  target_yaw += dt * target.v_yaw;

  // Choose the best armor to shoot
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
    target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);

  double selected_delta_angle = 0.0;
  int idx =
    selectBestArmor(armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num,selected_delta_angle);
  auto chosen_armor_position = armor_positions.at(idx);
  if (chosen_armor_position.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  // Calculate yaw, pitch, distance
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy, yaw, pitch);
  double distance = chosen_armor_position.norm();

  // Initialize gimbal_cmd
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.tp = pitch * 180 / M_PI;
  gimbal_cmd.ty = yaw * 180 / M_PI;
  gimbal_cmd.fire_advice = isOnTarget(rpy[2], rpy[1], yaw, pitch, distance,
                                      target.armors_num, selected_delta_angle);

  // 如果目标角速度超过最大跟踪角速度，建议开火
  if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
    gimbal_cmd.fire_advice = true;
  }

  switch (state) {
    case TRACKING_ARMOR: {
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;
      }

      // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to move
      if (controller_delay_ != 0) {
        target_position.x() += controller_delay_ * target.velocity.x;
        target_position.y() += controller_delay_ * target.velocity.y;
        target_position.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position,
                                            target_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.dz,
                                            target.armors_num);
        chosen_armor_position = armor_positions.at(idx);
        if (chosen_armor_position.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
        }
        calcYawAndPitch(chosen_armor_position, rpy, yaw, pitch);
      }
      break;
    }
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_ ) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      // 自转时撤销 z 速度预测，避免偏高；使用车体中心高度
      // target_position.z() -= dt * target.velocity.z;
      // target_position.z() += target.dz / 2.0;
      // calcYawAndPitch(target_position, rpy, yaw, pitch);
      break;
    }
  }
  // 应用瞄准偏移（与 launch rpy 解耦）
  yaw += yaw_offset_ * M_PI / 180.0;
  pitch += pitch_offset_ * M_PI / 180.0;

  gimbal_cmd.yaw = yaw * 180 / M_PI;
  gimbal_cmd.pitch = pitch * 180 / M_PI;
  gimbal_cmd.yaw_diff = (yaw - rpy[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (pitch - rpy[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance,
                        const size_t armors_num,
                        const double armor_delta_angle) const noexcept {
  // Armor-geometry-aware fire window
  // Use large armor width for BALANCE_2, small for others (approximation)
  double armor_half_w = (armors_num == 2) ? LARGE_ARMOR_HALF_W : SMALL_ARMOR_HALF_W;

  // Project armor width considering yaw inclination relative to camera LOS
  double cos_incidence = std::abs(std::cos(armor_delta_angle));
  double projected_half_w = armor_half_w * cos_incidence;

  // Angular tolerance = atan(projected_half_width / distance) * margin
  //fire_margin_ 允许你在理论几何尺寸基础上放大或缩小容差。
  double tolerance_rad = std::atan2(projected_half_w, distance) * fire_margin_;
  tolerance_rad = std::clamp(tolerance_rad, min_fire_tolerance_rad_, max_fire_tolerance_rad_);

  //采用最短角进行相减，避免角度周期性导致的误判
  bool on_target = std::abs(angles::shortest_angular_distance(cur_yaw, target_yaw)) < tolerance_rad;

  // Require two consecutive frames on target for stability
  bool stable = on_target && last_on_target_;
  last_on_target_ = on_target;

  return stable;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double dz,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = is_current_pair ? 0 : dz;
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = dz;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num,
                            double &selected_delta_angle) noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());

  // Compute delta_angle for each armor relative to camera-center line
  std::vector<double> delta_angles(armors_num);
  for (size_t i = 0; i < armors_num; i++) {
    double armor_yaw = target_yaw + i * (2 * M_PI / armors_num);
    // Normalize angle between armor face direction and camera-center line
    double delta = armor_yaw - alpha;
    // Normalize to [-pi, pi]
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;
    delta_angles[i] = delta;
  }

  int selected_id = 0;

  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    // Low speed: select armor with smallest |delta_angle|
    double min_abs_delta = std::abs(delta_angles[0]);
    for (size_t i = 1; i < armors_num; i++) {
      double abs_delta = std::abs(delta_angles[i]);
      if (abs_delta < min_abs_delta) {
        min_abs_delta = abs_delta;
        selected_id = static_cast<int>(i);
      }
    }
    // Lock mechanism: if the previous selection is still within 60°, keep it
    if (lock_id_ >= 0 && lock_id_ < static_cast<int>(armors_num)) {
      double lock_delta = std::abs(delta_angles[lock_id_]);
      if (lock_delta < M_PI / 3 && std::abs(lock_delta - min_abs_delta) < M_PI / 6) {
        selected_id = lock_id_;
      }
    }
  } else {
    // High speed spinning: use asymmetric coming/leaving angles
    double coming_rad = coming_angle_ / 180.0 * M_PI;
    double leaving_rad = leaving_angle_ / 180.0 * M_PI;

    double best_score = 1e9;
    for (size_t i = 0; i < armors_num; i++) {
      double delta = delta_angles[i];
      bool in_coming_zone;
      if (target_v_yaw < 0) {

        in_coming_zone = (delta > -leaving_rad && delta < coming_rad);
      } else {

        in_coming_zone = (delta > -coming_rad && delta < leaving_rad);
      }

      if (in_coming_zone) {
        double score = std::abs(delta);
        if (score < best_score) {
          best_score = score;
          selected_id = static_cast<int>(i);
        }
      }
    }

    // Fallback: if no armor in coming zone, pick closest
    if (best_score >= 1e9) {
      double min_abs_delta = 1e9;
      for (size_t i = 0; i < armors_num; i++) {
        double abs_delta = std::abs(delta_angles[i]);
        if (abs_delta < min_abs_delta) {
          min_abs_delta = abs_delta;
          selected_id = static_cast<int>(i);
        }
      }
    }
  }

  lock_id_ = selected_id;
  selected_delta_angle = delta_angles[selected_id];
  return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}
}  // namespace fyt::auto_aim
