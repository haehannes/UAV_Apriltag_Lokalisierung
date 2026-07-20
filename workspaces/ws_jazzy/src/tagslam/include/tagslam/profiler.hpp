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

#ifndef TAGSLAM__PROFILER_HPP_
#define TAGSLAM__PROFILER_HPP_

#include <chrono>
#include <iostream>
#include <unordered_map>

namespace tagslam
{
class Profiler
{
public:
  using TimePoint = std::chrono::high_resolution_clock::time_point;
  using Duration = std::chrono::microseconds;
  Profiler() {}
  virtual ~Profiler() {}

  void reset(const char * label)
  {
    const auto t = std::chrono::high_resolution_clock::now();
    ProfilerMap::iterator i = map_.find(label);
    if (i == map_.end()) {
      map_[label] = MapEntry(PTimer(), t);
    } else {
      i->second.lastTime = t;
    }
  }
  int record(const char * label, int ncount = 1)
  {
    ProfilerMap::iterator i = map_.find(label);
    if (i == map_.end()) {
      std::cout << "ERROR: invalid timer: " << label << std::endl;
      throw std::runtime_error("invalid timer!");
    }
    auto & me = i->second;
    const TimePoint now = std::chrono::high_resolution_clock::now();
    const Duration usec =
      std::chrono::duration_cast<Duration>(now - me.lastTime);
    me.timer = PTimer(usec, me.timer, ncount);
    me.lastTime = now;
    return (usec.count());
  }
  friend std::ostream & operator<<(std::ostream & os, const Profiler & p);

private:
  struct PTimer
  {
    PTimer();
    explicit PTimer(
      const Duration & d, const PTimer & oldTimer, int ncount = 1);
    Duration duration;   // sum of durations
    int64_t sqduration;  // sum of squared durations
    Duration min;        // smallest duration
    Duration max;        // largest duration
    int64_t count;       // number of samples
  };
  struct MapEntry
  {
    explicit MapEntry(
      const PTimer & p = PTimer(), const TimePoint & t = TimePoint())
    : timer(p), lastTime(t)
    {
    }
    PTimer timer;
    TimePoint lastTime;
  };
  using ProfilerMap = std::unordered_map<const char *, MapEntry>;
  ProfilerMap map_;
};
std::ostream & operator<<(std::ostream & os, const Profiler & p);
}  // namespace tagslam
#endif  // TAGSLAM__PROFILER_HPP_
