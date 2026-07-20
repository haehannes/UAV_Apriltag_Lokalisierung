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

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <tagslam/body.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/measurements/plane.hpp>
#include <tagslam/yaml.hpp>

namespace tagslam
{
namespace measurements
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("plane")); }

#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

void Plane::writeDiagnostics(const GraphPtr & graph)
{
  std::ofstream f("plane_diagnostics.txt");
  for (const auto & v : vertexes_) {
    if (graph->isOptimized(v)) {
      const auto p = factor::Coordinate::cast_const((*graph)[v]);
      const double l = factor::Coordinate::getOptimized(v, *graph);
      const double diff = l - p->getLength();
      f << FMT(6, 3) << graph->getError(v) << " diff: " << FMT(6, 3) << diff
        << " opt: " << FMT(6, 3) << l << " meas: " << FMT(6, 3)
        << p->getLength() << " " << *p << std::endl;
    }
  }
}
std::vector<FactorPtr> Plane::generate_factors(
  const std::string & name, double d, double noise, const Point3d & dir,
  const std::vector<int> & tags, TagFactory * tagFac)
{
  std::vector<FactorPtr> factors;
  for (const auto & tag : tags) {
    std::string coordName = name + "_tag_" + std::to_string(tag);
    TagConstPtr tagPtr = tagFac->findTag(tag);
    if (!tagPtr) {
      BOMB_OUT("measured tag is not valid: " << tag);
    }
    CoordinateFactorPtr fp(
      new factor::Coordinate(d, noise, dir, -1, tagPtr, coordName));
    factors.push_back(fp);
  }
  return (factors);
}

Plane::PlaneMeasurementsPtr Plane::read(
  const YAML::Node & config, TagFactory * tagFactory)
{
  if (!config["plane_measurements"]) {
    LOG_INFO("no plane measurements found!");
    return PlaneMeasurementsPtr();
  }
  const std::shared_ptr<measurements::Plane> m(new measurements::Plane());
  const auto meas = config["plane_measurements"];
  if (!meas.IsSequence() || meas.size() == 0) {
    BOMB_OUT("plane measurements must be valid array!");
  }
  LOG_INFO("found " << meas.size() << " plane measurement(s)!");
  for (size_t i = 0; i < meas.size(); i++) {  // iterate over planes
    try {
      if (!meas[i].IsMap()) {
        BOMB_OUT("plane " << i << " is not struct!");
      }
      if (meas[i].size() != 1) {
        BOMB_OUT("plane " << i << " has wrong number of fields");
      }
      const std::string name = meas[i][0].Tag();
      const auto plane = meas[i][0];
      const std::vector<int> tags =
        yaml::parse_container<std::vector<int>>(plane, "tags");
      const double d = yaml::parse<double>(plane, "distance");
      const double noise = yaml::parse<double>(plane, "noise");
      const Point3d dir = make_point(
        yaml::parse_container<std::vector<double>>(plane, "direction"));
      if (std::abs(dir.norm() - 1.0) > 1e-5) {
        BOMB_OUT("plane has non-unit direction");
      }
      LOG_INFO("found plane measurement: " << name);
      m->factors_ = generate_factors(name, d, noise, dir, tags, tagFactory);
    } catch (const std::runtime_error & e) {
      BOMB_OUT("config error with plane " << meas[i].begin()->first);
    }
  }
  return (m);
}
}  // namespace measurements
}  // namespace tagslam
