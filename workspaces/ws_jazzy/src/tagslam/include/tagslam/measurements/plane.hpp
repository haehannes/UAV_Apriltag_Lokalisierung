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

#ifndef TAGSLAM__MEASUREMENTS__PLANE_HPP_
#define TAGSLAM__MEASUREMENTS__PLANE_HPP_

#include <tagslam/geometry.hpp>
#include <tagslam/measurements/measurements.hpp>
#include <tagslam/vertex.hpp>
#include <vector>

namespace YAML
{
class Node;  // forward decl
}

namespace tagslam
{
namespace measurements
{
class Plane : public Measurements
{
public:
  typedef std::shared_ptr<Plane> PlaneMeasurementsPtr;

  void writeDiagnostics(const GraphPtr & graph) override;

  // static functions
  static PlaneMeasurementsPtr read(const YAML::Node & config, TagFactory * fac);

private:
  // static functions
  static std::vector<FactorPtr> generate_factors(
    const std::string & name, double d, double noise, const Point3d & dir,
    const std::vector<int> & tags, TagFactory * tagFac);
};
}  // namespace measurements
}  // namespace tagslam

#endif  // TAGSLAM__MEASUREMENTS__PLANE_HPP_
