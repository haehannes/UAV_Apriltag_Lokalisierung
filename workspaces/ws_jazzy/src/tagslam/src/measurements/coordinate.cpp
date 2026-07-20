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
#include <tagslam/measurements/coordinate.hpp>

namespace tagslam
{
namespace measurements
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("coordinate"));
}

#define FMT(X, Y) std::fixed << std::setw(X) << std::setprecision(Y)

void Coordinate::writeDiagnostics(const GraphPtr & graph)
{
  std::ofstream f("coordinate_diagnostics.txt");
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

Coordinate::CoordinateMeasurementsPtr Coordinate::read(
  const YAML::Node & config, TagFactory * tagFactory)
{
  if (!config["coordinate_measurements"]) {
    LOG_INFO("no coordinate measurements found!");
    return CoordinateMeasurementsPtr();
  }
  std::shared_ptr<measurements::Coordinate> m(new measurements::Coordinate());
  const auto meas = config["coordinate_measurements"];
  if (meas.IsSequence()) {
    auto fpts = factor::Coordinate::parse(meas, tagFactory);
    for (const auto & f : fpts) {
      if (!f->getTag()->getBody()->isStatic()) {
        BOMB_OUT("measured body must be static: " << *f);
      }
      LOG_INFO("found coordinate: " << *f);
      m->factors_.push_back(f);
    }
  } else {
    LOG_INFO("no coordinate measurements found!");
  }
  return (m);
}
}  // namespace measurements
}  // namespace tagslam
