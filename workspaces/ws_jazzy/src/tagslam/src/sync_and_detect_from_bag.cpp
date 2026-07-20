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

#include <rosbag2_transport/recorder.hpp>
#include <tagslam/enhanced_player.hpp>
#include <tagslam/logging.hpp>
#include <tagslam/sync_and_detect.hpp>

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("sync_and_detect_from_bag"));
}

static std::vector<std::string> merge(
  const std::vector<std::vector<std::string>> & v)
{
  std::vector<std::string> all;
  for (const auto & vv : v) {
    all.insert(all.end(), vv.begin(), vv.end());
  }
  return (all);
}

static void printTopics(
  const std::string & label, const std::vector<std::string> & topics)
{
  for (const auto & t : topics) {
    LOG_INFO(label << ": " << t);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);

  auto sync_node = std::make_shared<tagslam::SyncAndDetect>(node_options);
  exec.add_node(sync_node);

  const auto recorded_topics =
    merge({sync_node->getTagTopics(), sync_node->getSyncedOdomTopics()});
  std::cout << "tag topics: " << sync_node->getTagTopics().size() << std::endl;
  std::cout << "recorded topics: " << recorded_topics.size() << std::endl;
  printTopics("recorded topic", recorded_topics);

  const std::string in_uri =
    sync_node->declare_parameter<std::string>("in_bag", "");
  if (in_uri.empty()) {
    LOG_ERROR("must provide valid in_bag parameter!");
    return (-1);
  }
  LOG_INFO("using input bag: " << in_uri);
  rclcpp::NodeOptions player_options;
  using Parameter = rclcpp::Parameter;
  player_options.parameter_overrides(
    {Parameter("storage.uri", in_uri),
     Parameter("play.clock_publish_on_topic_publish", true),
     Parameter("play.start_paused", true), Parameter("play.rate", 1000.0),
     // Parameter("play.topics", in_topics),
     Parameter("play.disable_keyboard_controls", true)});
  auto player_node =
    std::make_shared<tagslam::EnhancedPlayer>("rosbag_player", player_options);
  exec.add_node(player_node);

  if (!player_node->hasImageTopics(sync_node->getImageTopics())) {
    LOG_ERROR("topics not in bag, sync will hang!");
  } else {
    LOG_INFO("player found all topics!");
  }

  const std::string out_uri =
    sync_node->declare_parameter<std::string>("out_bag", "");

  std::shared_ptr<rosbag2_transport::Recorder> recorder_node;
  size_t num_frames =
    sync_node->declare_parameter<int>("max_number_of_frames", 0);
  if (!out_uri.empty()) {
    LOG_INFO("writing detected tags to bag: " << out_uri);
    rclcpp::NodeOptions recorder_options;
    recorder_options.parameter_overrides(
      {Parameter("storage.uri", out_uri),
       Parameter("record.disable_keyboard_controls", true),
       Parameter("record.topics", recorded_topics)});

    recorder_node = std::make_shared<rosbag2_transport::Recorder>(
      "rosbag_recorder", recorder_options);
    exec.add_node(recorder_node);
  } else {
    LOG_INFO("no out_bag parameter set, publishing as messages!");
  }

  while (player_node->play_next() && rclcpp::ok()) {
    exec.spin_some();
    if (num_frames && sync_node->getNumberOfFrames() > num_frames) {
      LOG_INFO("reached max number of frames: " << num_frames);
      break;
    }
  }
  LOG_INFO("sync_and_detect finished!");
  rclcpp::shutdown();
  return 0;
}
