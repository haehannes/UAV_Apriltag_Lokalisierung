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
from collections import defaultdict

import rospy

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='produce squash section from error map projection factors.'
    )
    parser.add_argument(
        '--in_file',
        '-i',
        action='store',
        required=True,
        help='input file with outliers from error_map.txt',
    )
    parser.add_argument(
        '--out_file',
        '-o',
        action='store',
        required=True,
        help='file with generated squash section',
    )
    args = parser.parse_args(rospy.myargv()[1:])

    squash = defaultdict(lambda: defaultdict(lambda: list()))

    with open(args.in_file, 'r') as in_file:
        lines = in_file.read().splitlines()
        for line in lines:
            a = line.split()
            label = a[2].split(',')[0]
            tag = label.split(':')[2]
            cam = label.split(':')[1].split('-')[0]
            squash[a[1]][cam].append(tag)
            print('read tag: %s %s %s' % (a[1], cam, tag))

    with open(args.out_file, 'w') as out_file:
        out_file.write('squash:\n')
        for t, cams in squash.items():
            for c, s in cams.items():  # loop over cameras
                out_file.write('  - time: "%s"\n' % t)
                out_file.write('    camera: %s\n' % c)
                out_file.write('    tags: [')
                for i in range(0, len(s) - 1):
                    out_file.write('%s, ' % (s[i]))
                out_file.write('%s]\n' % s[-1])
