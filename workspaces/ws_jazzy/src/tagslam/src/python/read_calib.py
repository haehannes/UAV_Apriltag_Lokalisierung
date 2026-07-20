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

# read calibration files

import geometry_msgs
import numpy as np
from tf.transformations import identity_matrix
from tf.transformations import quaternion_from_matrix
from tf.transformations import rotation_matrix
import yaml


def read_yaml(filename):
    with open(filename, 'r') as y:
        try:
            return yaml.load(y)
        except yaml.YAMLError as e:
            print(e)


def matrix_to_tf(T):
    q = quaternion_from_matrix(T)
    tf = geometry_msgs.msg.Transform()
    tf.translation.x = T[0, 3]
    tf.translation.y = T[1, 3]
    tf.translation.z = T[2, 3]
    tf.rotation.x = q[0]
    tf.rotation.y = q[1]
    tf.rotation.z = q[2]
    tf.rotation.w = q[3]
    return tf


def rvec_tvec_to_mat(rvec, tvec):
    L = np.linalg.norm(rvec)
    n = rvec / L if L > 1e-8 else np.array([1.0, 0.0, 0.0])
    T = rotation_matrix(L, n)
    T[0:3, 3] = tvec
    return T


def read_calib(fname, use_imu_tf=False):
    c = read_yaml(fname)
    tfmd = {}
    for i in c.items():
        name, cam = i[0], i[1]
        if name == 'T_imu_body':
            continue
        tf = cam['T_cam_imu'] if name == 'cam0' else cam['T_cn_cnm1']
        T = np.asarray(tf)
        if name == 'cam0' and not use_imu_tf:
            T = identity_matrix()
        tfmd[name] = T

    T = identity_matrix()  # T_-1_w
    tfms = {}
    #    print tfmd

    for name in sorted(tfmd.keys()):
        # compute T_n_n-1 by premult T_n-1_w
        T = np.matmul(tfmd[name], T)
        Ti = np.linalg.inv(T)
        tfm = matrix_to_tf(Ti)
        tfms[name] = tfm
    return tfms
