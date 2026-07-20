
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
#include <tagslam/measurements/distance.hpp>

namespace tagslam
{
namespace measurements
{
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("distance")); }

#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

void Distance::writeDiagnostics(const GraphPtr & graph)
{
  std::ofstream f("distance_diagnostics.txt");
  for (const auto & v : vertexes_) {
    const auto p = factor::Distance::cast_const((*graph)[v]);
    if (graph->isOptimized(v)) {
      const double l = factor::Distance::getOptimized(v, *graph);
      const double diff = l - p->getDistance();
      f << FMT(6, 3) << graph->getError(v) << " diff: " << FMT(6, 3) << diff
        << " opt: " << FMT(6, 3) << l << " meas: " << FMT(6, 3)
        << p->getDistance() << " " << *p << std::endl;
    }
  }
}

Distance::DistanceMeasurementsPtr Distance::read(
  const YAML::Node & config, TagFactory * tagFactory)
{
  if (!config["distance_measurements"]) {
    LOG_INFO("no distance measurements found!");
    return DistanceMeasurementsPtr();
  }
  std::shared_ptr<measurements::Distance> m(new measurements::Distance());
  const auto meas = config[std::string("distance_measurements")];
  if (meas.IsSequence()) {
    auto factors = factor::Distance::parse(meas, tagFactory);
    for (const auto & f : factors) {
      if (
        !f->getTag(0)->getBody()->isStatic() ||
        !f->getTag(1)->getBody()->isStatic()) {
        BOMB_OUT("measured bodies must be static: " << *f);
      }
      LOG_INFO("found distance: " << *f);
      m->factors_.push_back(f);
    }
  } else {
    LOG_INFO("no distance measurements found!");
  }
  return (m);
}
}  // namespace measurements
}  // namespace tagslam
