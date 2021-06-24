# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os

from setuptools import setup, find_packages

dir_path = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(dir_path,"gpmp2_python", "version.py")) as fp:
    exec(fp.read())

def read_requirements_file(filename):
    req_file_path = "%s/%s" % (dir_path, filename)
    with open(req_file_path) as f:
        return [line.strip() for line in f]

requirements_file = "requirements_python2.txt"

setup(
    name="gpmp2_python",
    version=__version__,
    author="Kalyan Vasudev",
    url="https://github.com/fairinternal/gpmp2_python.git",
    license="MIT",
    packages=find_packages(exclude=("scripts", "tests")),
    install_requires=read_requirements_file(requirements_file),
)
