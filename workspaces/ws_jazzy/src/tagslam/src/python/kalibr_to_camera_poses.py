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
import math

import numpy as np
import read_calib
import rospy
from tf.transformations import rotation_matrix

R = np.asarray(
    [
        1000000.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1000000.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1000000.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1000000.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1000000.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1000000.0,
    ]
)


def quaternion_to_axis_angle(q):
    a = 2.0 * math.acos(q.w)
    sqinv = 1.0 / math.sqrt(1.0 - q.w * q.w) if q.w * q.w < 1.0 - 1e-8 else 0
    aa = a * np.asarray((q.x, q.y, q.z)) * sqinv
    return aa


def rvec_tvec_to_mat(rvec, tvec):
    l = np.linalg.norm(rvec)
    n = rvec / l if l > 1e-8 else np.array([1.0, 0.0, 0.0])
    T = rotation_matrix(l, n)
    T[0:3, 3] = tvec
    return T


def print_item(name, tf):
    aa = quaternion_to_axis_angle(tf.rotation)
    print('%s:' % name)
    print('  pose:')
    print('    position:')
    print('      x: ', tf.translation.x)
    print('      y: ', tf.translation.y)
    print('      z: ', tf.translation.z)
    print('    rotation:')
    print('      x: ', aa[0])
    print('      y: ', aa[1])
    print('      z: ', aa[2])
    print('    R:')
    print('      [', ('{:.8f}, ' * 6).format(*R[0:6]))
    for i in range(0, 4):
        print('       ', ('{:.8f}, ' * 6).format(*R[(i * 6 + 6) : (i * 6 + 12)]))
    print('       ', ('{:.8f}, ' * 5).format(*R[30:35]), '%.8f]' % R[35])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='convert kalibr to camera_poses.yaml')
    parser.add_argument(
        '--use_imu_tf', '-i', action='store', default=False, type=bool, help='use imu transform.'
    )
    parser.add_argument(
        '--calib', action='store', default=None, required=True, help='name of calibration file'
    )

    args = parser.parse_args()

    tfs = read_calib.read_calib(args.calib, args.use_imu_tf)
    for name in sorted(tfs.keys()):
        print_item(name, tfs[name])
