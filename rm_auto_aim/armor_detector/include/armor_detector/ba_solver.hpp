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

#ifndef ARMOR_DETECTOR_BA_SOLVER_HPP_
#define ARMOR_DETECTOR_BA_SOLVER_HPP_

// std
#include <array>
#include <cstddef>
#include <deque>
#include <vector>
// 3rd party
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
// project
#include "armor_detector/types.hpp"

namespace fyt::auto_aim {

// Two-stage search based Yaw optimizer replacing G2O Bundle Adjustment.
// Stage 1: Coarse search over 360° with 1° steps
// Stage 2: Fine search ±2° around best coarse result with 0.1° steps
// More robust than BA — does not depend on PnP initialization quality.
class BaSolver {
public:
  BaSolver(std::array<double, 9> &camera_matrix, std::vector<double> &dist_coeffs);

  // Solve the armor yaw using two-stage search, modifies rmat in-place
  bool solveBa(const std::deque<Armor> &armors, cv::Mat &rmat) noexcept;

private:
  // Compute reprojection error for a given yaw angle
  double computeReprojError(const Eigen::Matrix3d &camera2imu,
                            const Eigen::Vector3d &tvec,
                            const std::vector<cv::Point2f> &landmarks,
                            const std::vector<Eigen::Vector3d> &object_points,
                            double pitch,
                            double yaw) const noexcept;

  CameraInternalK cam_internal_k_;
};

}  // namespace fyt::auto_aim
#endif  // ARMOR_DETECTOR_BA_SOLVER_HPP_
