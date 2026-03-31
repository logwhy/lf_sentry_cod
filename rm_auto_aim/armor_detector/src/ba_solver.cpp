// Created by Labor 2023.8.25
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

#include "armor_detector/ba_solver.hpp"
// std
#include <cmath>
// 3rd party
#include <Eigen/Core>
// project
#include "armor_detector/types.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {

BaSolver::BaSolver(std::array<double, 9> &camera_matrix, std::vector<double> &dist_coeffs) {
  cam_internal_k_ = CameraInternalK{
    .fx = camera_matrix[0], .fy = camera_matrix[4], .cx = camera_matrix[2], .cy = camera_matrix[5]};
}

double BaSolver::computeReprojError(const Eigen::Matrix3d &camera2imu,
                                    const Eigen::Vector3d &tvec,
                                    const std::vector<cv::Point2f> &landmarks,
                                    const std::vector<Eigen::Vector3d> &object_points,
                                    double pitch,
                                    double yaw) const noexcept {
  // Construct rotation: IMU → Armor with fixed pitch and candidate yaw
  std::array<double, 3> euler = {0, pitch, yaw};
  Eigen::Matrix3d imu2armor = utils::eulerToMatrix(euler, utils::EulerOrder::XYZ);

  // Camera → IMU → Armor
  Eigen::Matrix3d R = camera2imu * imu2armor;
  Eigen::Matrix3d K = cam_internal_k_.toMatrix();

  double total_error = 0.0;
  for (size_t i = 0; i < object_points.size(); i++) {
    // Project 3D object point to image
    Eigen::Vector3d p = R * object_points[i] + tvec;
    if (p.z() <= 0) return 1e9;  // Behind camera
    p = K * (p / p.z());

    double dx = p.x() - landmarks[i].x;
    double dy = p.y() - landmarks[i].y;
    total_error += dx * dx + dy * dy;
  }
  return total_error;
}

bool BaSolver::solveBa(const std::deque<Armor> &armors, cv::Mat &rmat) noexcept {
  if (armors.empty()) {
    return true;
  }

  const auto &armor = armors.back();
  auto landmarks = armor.landmarks();

  // Coordinate transforms
  Eigen::Matrix3d camera2imu = armor.imu2camera.transpose();
  Eigen::Vector3d tvec(armor.tvec.at<double>(0), armor.tvec.at<double>(1), armor.tvec.at<double>(2));

  // Object points and pitch prior
  auto armor_size = armor.type == ArmorType::SMALL
                      ? Eigen::Vector2d(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)
                      : Eigen::Vector2d(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT);
  auto object_points = Armor::buildObjectPoints<Eigen::Vector3d>(armor_size(0), armor_size(1));
  double pitch = armor.number == "outpost" ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;

  // --- Stage 1: Coarse search over 360° with 1° steps ---
  double best_yaw = 0.0;
  double min_error = 1e9;
  constexpr int COARSE_STEPS = 360;
  constexpr double COARSE_STEP_RAD = CV_PI / 180.0;  // 1 degree

  for (int i = 0; i < COARSE_STEPS; i++) {
    double yaw = -CV_PI + i * COARSE_STEP_RAD;
    double error = computeReprojError(camera2imu, tvec, landmarks, object_points, pitch, yaw);
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // --- Stage 2: Fine search ±2° around best with 0.1° steps ---
  constexpr double FINE_RANGE_RAD = 2.0 * CV_PI / 180.0;
  constexpr double FINE_STEP_RAD = 0.1 * CV_PI / 180.0;
  double fine_start = best_yaw - FINE_RANGE_RAD;
  double fine_end = best_yaw + FINE_RANGE_RAD;

  for (double yaw = fine_start; yaw <= fine_end; yaw += FINE_STEP_RAD) {
    double error = computeReprojError(camera2imu, tvec, landmarks, object_points, pitch, yaw);
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // Construct optimized rotation matrix
  std::array<double, 3> best_euler = {0, pitch, best_yaw};
  Eigen::Matrix3d imu2armor = utils::eulerToMatrix(best_euler, utils::EulerOrder::XYZ);
  Eigen::Matrix3d rmat_optimized = armor.imu2camera.transpose().transpose() * imu2armor;
  // imu2camera^T^T = imu2camera, so: rmat_optimized = imu2camera * imu2armor
  // But we need camera2armor = (imu2camera)^T * imu2armor
  rmat_optimized = armor.imu2camera.transpose() * imu2armor;

  // Convert Eigen to cv::Mat
  rmat = utils::eigenToCv(rmat_optimized);

  return true;
}

}  // namespace fyt::auto_aim
