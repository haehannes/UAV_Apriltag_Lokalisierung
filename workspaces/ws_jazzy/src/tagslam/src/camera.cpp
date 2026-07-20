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

#include <tagslam/camera.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/yaml.hpp>

namespace tagslam
{
using std::string;
static rclcpp::Logger get_logger() { return (rclcpp::get_logger("camera")); }

CameraPtr Camera::parse_camera(const string & name, const YAML::Node & config)
{
  CameraPtr camPtr(new Camera());
  Camera & cam = *camPtr;  // short hand
  cam.name_ = name;
  cam.intrinsics_ = CameraIntrinsics::parse(config);
  cam.imageTopic_ = yaml::parse<string>(config, "image_topic");
  if (config["image_transport"]) {
    cam.imageTransport_ = yaml::parse<string>(config, "image_transport");
  }
  cam.tagTopic_ = yaml::parse<string>(config, "tag_topic", "");
  cam.rigName_ = yaml::parse<string>(config, "rig_body");
  cam.frameId_ = yaml::parse<string>(config, "frame_id", cam.name_);
  double wiggleR = yaml::parse<double>(config, "wiggle_rotation", 0.00001);
  double wiggleT = yaml::parse<double>(config, "wiggle_translation", 0.00001);
  cam.wiggle_ = PoseNoise::make(wiggleR, wiggleT);
  return (camPtr);
}

CameraVec Camera::parse_cameras(const YAML::Node & config)
{
  CameraVec cdv;
  int cam_idx = 0;
  for (size_t cam_num = 0; cam_num < 100; cam_num++) {
    string name = "cam" + std::to_string(cam_num);
    if (!config[name]) {
      name = "camera_" + std::to_string(cam_num);
      if (!config[name]) {
        continue;
      }
    }
    try {
      CameraPtr camera = parse_camera(name, config[name]);
      camera->index_ = cam_idx++;
      cdv.push_back(camera);
    } catch (const std::runtime_error & e) {
      BOMB_OUT("error reading camera: " << name << " " << e.what());
    }
  }
  return (cdv);
}

}  // namespace tagslam
