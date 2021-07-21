import os

from setuptools import setup, find_packages

dir_path = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(dir_path,"gpmp2_python", "version.py")) as fp:
    exec(fp.read())

def read_requirements_file(filename):
    req_file_path = "%s/%s" % (dir_path, filename)
    with open(req_file_path) as f:
        return [line.strip() for line in f]

requirements_file = "requirements.txt"

setup(
    name="gpmp2_python",
    version=__version__,
    packages=find_packages(exclude=("scripts", "tests")),
    install_requires=read_requirements_file(requirements_file)
)
