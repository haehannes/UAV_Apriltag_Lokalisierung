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

#include <sstream>
#include <string>
#include <tagslam/body.hpp>
#include <tagslam/factor/coordinate.hpp>
#include <tagslam/graph.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/yaml.hpp>

namespace tagslam
{
namespace factor
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("coordinate_factor"));
}

Coordinate::Coordinate(
  double len, double noise, const Point3d & direction, const int corn,
  const TagConstPtr & tag, const string & name)
: Factor(name, 0ULL),
  length_(len),
  noise_(noise),
  direction_(direction),
  corner_(corn),
  tag_(tag)
{
}

VertexDesc Coordinate::addToGraph(const VertexPtr & vp, Graph * g) const
{
  const uint64_t t0{0ULL};
  const VertexDesc vtp = g->findTagPose(getTag()->getId());
  checkIfValid(vtp, "no tag pose found");
  const VertexDesc vbp = g->findBodyPose(t0, getTag()->getBody()->getName());
  checkIfValid(vbp, "no body pose found");
  const VertexDesc fv = g->insertFactor(vp);
  g->addEdge(fv, vbp, 0);
  g->addEdge(fv, vtp, 1);
  return (fv);
}

void Coordinate::addToOptimizer(Graph * g) const
{
  const VertexDesc v = g->find(this);
  checkIfValid(v, "factor not found");
  const std::vector<ValueKey> optKeys = g->getOptKeysForFactor(v, 2);
  const FactorKey fk = g->getOptimizer()->addCoordinateMeasurement(
    getLength(), getNoise(), getDirection(), getCorner(),
    optKeys[0] /* T_w_b */, optKeys[1] /* T_b_o */);
  g->markAsOptimized(v, fk);
}

double Coordinate::coordinate(
  const Transform & T_w_b, const Transform & T_b_o) const
{
  const auto X = T_w_b * T_b_o * getCorner();
  return (direction_.dot(X));
}

const Eigen::Vector3d Coordinate::getCorner() const
{
  return (tag_->getObjectCorner(corner_));
}

CoordinateFactorPtr Coordinate::parse(
  const string & name, const YAML::Node & meas, TagFactory * tagFactory)
{
  int tag(-1), c(-1);
  double len(-1e10), noise(-1);
  Eigen::Vector3d dir(0.0, 0.0, 0.0);
  try {
    tag = yaml::parse<int>(meas, "tag");
    c = yaml::parse<int>(meas, "corner");
    len = yaml::parse<double>(meas, "length");
    noise = yaml::parse<double>(meas, "noise");
    dir =
      make_point(yaml::parse_container<std::vector<double>>(meas, "direction"));
  } catch (const std::runtime_error & e) {
    BOMB_OUT("error parsing measurement: " << name << ": " << e.what());
  }
  if (std::abs(dir.norm() - 1.0) > 1e-5) {
    BOMB_OUT("measurement " + name + " has non-unit direction");
  }
  CoordinateFactorPtr fp;
  if (tag >= 0 && c >= 0 && len > -1e10 && noise > 0) {
    TagConstPtr tagPtr = tagFactory->findTag(tag);
    if (!tagPtr) {
      LOG_WARN("ignoring unknown measured tag: " << name);
      return CoordinateFactorPtr();
    }
    fp.reset(new factor::Coordinate(len, noise, dir, c, tagPtr, name));
  } else {
    BOMB_OUT("coordinate measurement incomplete: " << name);
  }
  return (fp);
}

CoordinateFactorPtrVec Coordinate::parse(
  const YAML::Node & meas, TagFactory * tagFactory)
{
  CoordinateFactorPtrVec fv;
  if (!meas.IsSequence()) {
    BOMB_OUT("invalid node type for coordinate measurements!");
  }
  for (size_t i = 0; i < meas.size(); i++) {
    if (!meas[i].IsMap()) continue;
    for (const auto & m : meas[i]) {
      if (m.IsMap()) {
        CoordinateFactorPtr d = parse(m.Tag(), m, tagFactory);
        if (d) {
          fv.push_back(d);
        }
      }
    }
  }
  return (fv);
}

string Coordinate::getLabel() const
{
  std::stringstream ss;
  ss << name_;
  return (ss.str());
}
// static function!
double Coordinate::getOptimized(const VertexDesc & v, const Graph & g)
{
  if (!g.isOptimized(v)) {
    return (-1.0);  // not optimized yet!
  }
  const auto p = std::dynamic_pointer_cast<const factor::Coordinate>(g[v]);
  if (!p) {
    BOMB_OUT("vertex is not coord: " << *g[v]);
  }
  const std::vector<ValueKey> optKeys = g.getOptKeysForFactor(v, 2);
  const auto opt = g.getOptimizer();
  const double l =
    p->coordinate(opt->getPose(optKeys[0]), opt->getPose(optKeys[1]));
  return (l);
}
}  // namespace factor
}  // namespace tagslam
