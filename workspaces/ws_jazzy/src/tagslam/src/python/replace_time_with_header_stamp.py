#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

#
# replace ros time stamp with header time stamp
#
# lifted from the ROS cook book: http://wiki.ros.org/rosbag/Cookbook
#

import argparse

import rosbag

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='replace ros time with header stamp',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        '--out_bag', '-o', action='store', default='out.bag', help='name of the output bag.'
    )
    parser.add_argument(
        '--in_bag',
        '-i',
        action='store',
        default=None,
        required=True,
        help='name of the input bag.',
    )

    args = parser.parse_args(rospy.myargv()[1:])

    with rosbag.Bag(args.out_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(args.in_bag).read_messages():
            if topic == '/tf' and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
