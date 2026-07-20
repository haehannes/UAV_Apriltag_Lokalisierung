// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <string>
#include <tagslam/geometry.hpp>
#include <tagslam/pose_noise.hpp>

namespace tagslam
{
namespace yaml_utils
{
using std::string;
void write_matrix(
  std::ostream & of, const string & prefix, const Transform & pose);
void write_pose(
  std::ostream & of, const string & prefix, const Transform & pose,
  const PoseNoise & n, bool writeNoise);
void write_pose_with_covariance(
  std::ostream & of, const string & prefix, const Transform & pose,
  const PoseNoise & n);
template <typename T>
void write_container(
  std::ostream & of, const string & pf, T c, int w = 12, int p = 8)
{
  (void)pf;
  of << "[";
  for (int i = 0; i < static_cast<int>(c.size()) - 1; i++) {
    of << std::fixed << std::setw(w) << std::setprecision(p);
    of << c[i] << ",";
  }
  if (!c.empty()) {
    of << std::fixed << std::setw(w) << std::setprecision(p);
    of << c[c.size() - 1];
  }
  of << "]";
}
}  // namespace yaml_utils
}  // namespace tagslam
