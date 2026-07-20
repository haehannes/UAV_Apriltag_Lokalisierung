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

#include <tagslam/logging.hpp>
#include <tagslam/measurements/coordinate.hpp>
#include <tagslam/measurements/distance.hpp>
#include <tagslam/measurements/measurements.hpp>
#include <tagslam/measurements/plane.hpp>

namespace tagslam
{
namespace measurements
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("measurements"));
}

std::vector<MeasurementsPtr> read_all(
  const YAML::Node & config, TagFactory * tagFactory)
{
  std::vector<MeasurementsPtr> meas;
  MeasurementsPtr m;

  m = measurements::Distance::read(config, tagFactory);
  if (m) {
    meas.push_back(m);
  }

  m = measurements::Coordinate::read(config, tagFactory);
  if (m) {
    meas.push_back(m);
  }

  m = measurements::Plane::read(config, tagFactory);
  if (m) {
    meas.push_back(m);
  }

  return (meas);
}

void Measurements::addToGraph(const GraphPtr & graph)
{
  for (const auto & f : factors_) {
    vertexes_.push_back(f->addToGraph(f, graph.get()));
  }
}

void Measurements::tryAddToOptimizer(const GraphPtr & graph)
{
  for (const auto & v : vertexes_) {
    if (graph->isOptimizableFactor(v) && !graph->isOptimized(v)) {
      auto fp = std::dynamic_pointer_cast<factor::Factor>((*graph)[v]);
      fp->addToOptimizer(graph.get());
    }
  }
}

void Measurements::printUnused(const GraphConstPtr & graph)
{
  for (const auto & f : vertexes_) {
    if (!graph->isOptimized(f)) {
      LOG_INFO("unused measurement: " << (*graph)[f]);
    }
  }
}

}  // namespace measurements
}  // namespace tagslam
