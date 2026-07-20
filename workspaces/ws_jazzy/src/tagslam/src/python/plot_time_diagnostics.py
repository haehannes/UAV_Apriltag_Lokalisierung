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
# plot error as a function of time
#

import argparse
from os.path import expanduser

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='plot error as function of time')
    parser.add_argument(
        '--file',
        '-f',
        action='store',
        default=expanduser('~') + '/.ros/time_diagnostics.txt',
        help='the time diagnostics file.',
    )
    args = parser.parse_args()

    v = []
    with open(args.file) as file:
        for line in file.readlines():
            arr = line.strip().split(' ')
            v.append([float(x) for x in arr[0:2]])
    va = np.array(v)
    plt.plot(va[1:, 0], va[1:, 1])
    plt.show()
