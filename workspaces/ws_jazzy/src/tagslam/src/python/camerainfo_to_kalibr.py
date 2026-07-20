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

import argparse
import os


import yaml


def write_cameras_file(c, f):
    K = c['camera_matrix']['data']
    D = c['distortion_coefficients']['data']
    f.write('cam0:\n')
    f.write('  camera_model: pinhole\n')
    f.write('  intrinsics: [%.5f, %.5f, %.5f, %.5f]\n' % (K[0], K[4], K[2], K[5]))
    f.write('  distortion_model: %s\n' % c['distortion_model'])
    f.write('  distortion_coeffs: [%.5f, %.5f, %.5f, %.5f, %.5f]\n' % tuple(D))
    f.write('  resolution: [%d, %d]\n' % (c['image_width'], c['image_height']))
    f.write('  image_topic: /%s/tags\n' % (c['camera_name']))
    f.write('  tag_topic: /%s/tags\n' % (c['camera_name']))
    f.write('  rig_body: XXX_NAME_OF_RIG_BODY\n')


def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='convert ROS style format to calibr.')
    parser.add_argument(
        '--in_file', '-i', action='store', required=True, help='ROS calibration input yaml file'
    )
    parser.add_argument(
        '--out_file',
        '-o',
        action='store',
        default=None,
        required=False,
        help='TagSLAM output file cameras.yaml',
    )
    args = parser.parse_args()
    ros_calib = read_yaml(args.in_file)
    if args.out_file is None:
        file_and_ext = os.path.splitext(args.in_file)
        args.out_file = file_and_ext[0] + '_kalibr' + file_and_ext[1]
    with open(args.out_file, 'w') as f:
        write_cameras_file(ros_calib, f)
