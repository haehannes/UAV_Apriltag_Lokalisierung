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

import numpy as np
import tf
import yaml


def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)


def rvec_tvec_to_mat(rvec, tvec):
    l = np.linalg.norm(rvec)
    n = rvec / l if l > 1e-8 else np.array([1.0, 0.0, 0.0])
    T = tf.transformations.rotation_matrix(l, n)
    T[0:3, 3] = tvec
    return T


def print_tf(T):
    for i in range(0, 4):
        x = T[i, :]
        print('  - [%15.12f, %15.12f, %15.12f, %15.12f]' % (x[0], x[1], x[2], x[3]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='convert camera_poses.yaml to kalibr transform')
    parser.add_argument(
        '--camera_poses',
        '-c',
        action='store',
        default=None,
        required=True,
        help='name of camera_poses.yaml file',
    )

    args = parser.parse_args()

    y = read_yaml(args.camera_poses)
    T_w_cnm1 = np.eye(4)
    for cam in sorted(y.keys()):
        p = y[cam]['pose']['position']
        pos = np.asarray([p['x'], p['y'], p['z']])
        r = y[cam]['pose']['rotation']
        rvec = np.asarray([r['x'], r['y'], r['z']])
        T_w_cn = rvec_tvec_to_mat(rvec, pos)
        print(cam)
        print('  T_cn_cnm1:')
        print_tf(np.matmul(np.linalg.inv(T_w_cn), T_w_cnm1))
        T_w_cnm1 = T_w_cn
