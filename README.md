**Note**: Version compatible with latest GTSAM is being maintained at [borglab/gpmp2](https://github.com/borglab/gpmp2).

GPMP2
===================================================

This library is an implementation of GPMP2 (Gaussian Process Motion Planner 2) algorithm described in [Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs](http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf) (RSS 2016). The core library is developed in C++ language with an optional Python 2.7 toolbox. GPMP2 was started at the Georgia Tech Robot Learning Lab, see [THANKS](THANKS.md) for contributors.


Prerequisites
------

- CMake >= 3.0 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.50 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [Anaconda2](https://docs.anaconda.com/anaconda/install/linux/), virtual environment needed if installing python toolbox.
- [GTSAM](https://github.com/borglab/gtsam/tree/wrap-export) == `wrap_export`, a C++ library that implements smoothing and mapping (SAM) framework in robotics and vision. Here we use the factor graph implementations and inference/optimization tools provided by GTSAM.


Installation (C++ only)
------

- Install GTSAM.
  ```bash
  git clone https://github.com/borglab/gtsam.git
  cd gtsam
  git checkout wrap-export
  mkdir build && cd build
  cmake ..
  make check  # optional, run unit tests
  sudo make install
  ```
- Setup paths.
  ```bash
  echo 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=/usr/local/share:${LD_LIBRARY_PATH}' >> ~/.bashrc
  source ~/.bashrc
  ```
- Install gpmp2.
  ```bash
  git clone https://github.com/gtrll/gpmp2.git
  cd gpmp2 && mkdir build && cd build
  cmake ..
  make check  # optional, run unit tests
  sudo make install
  ```


Installation (C++ with Python toolbox)
------
- Setup virtual environment.
  ```bash
  conda create -n gpmp2 pip python=2.7
  conda activate gpmp2
  pip install cython numpy scipy matplotlib
  conda deactivate
  ```
- Install GTSAM.
  ```bash
  conda activate gpmp2
  git clone https://github.com/borglab/gtsam.git
  cd gtsam
  git checkout wrap-export
  mkdir build && cd build
  cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX:=ON ..
  make check  # optional, run unit tests
  sudo make install
  conda deactivate
  ```
- Setup paths.
  ```bash
  echo 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
  echo 'export LD_LIBRARY_PATH=/usr/local/share:${LD_LIBRARY_PATH}' >> ~/.bashrc
  echo 'export PYTHONPATH=/usr/local/cython:${PYTHONPATH}' >> ~/.bashrc
  source ~/.bashrc
  ```
- Install gpmp2.
  ```bash
  conda activate gpmp2
  git clone https://github.com/gtrll/gpmp2.git
  cd gpmp2 && mkdir build && cd build
  cmake -DGPMP2_BUILD_PYTHON_TOOLBOX:=ON ..
  make check  # optional, run unit tests
  sudo make install
  cd ../gpmp2_python && pip install -e .
  conda deactivate
  ```


Citing
-----

If you use GPMP2 in an academic context, please cite following publications:

```
@inproceedings{Mukadam-IJRR-18,
  Author = {Mustafa Mukadam and Jing Dong and Xinyan Yan and Frank Dellaert and Byron Boots},
  Title = {Continuous-time {G}aussian Process Motion Planning via Probabilistic Inference},
  journal = {The International Journal of Robotics Research (IJRR)},
  volume = {37},
  number = {11},
  pages = {1319--1340},
  year = {2018}
}

@inproceedings{Dong-RSS-16,
  Author = {Jing Dong and Mustafa Mukadam and Frank Dellaert and Byron Boots},
  Title = {Motion Planning as Probabilistic Inference using {G}aussian Processes and Factor Graphs},
  booktitle = {Proceedings of Robotics: Science and Systems (RSS)},
  year = {2016}
}

@inproceedings{dong2018sparse,
  title={Sparse {G}aussian Processes on Matrix {L}ie Groups: A Unified Framework for Optimizing Continuous-Time Trajectories},
  author={Dong, Jing and Mukadam, Mustafa and Boots, Byron and Dellaert, Frank},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={6497--6504},
  year={2018},
  organization={IEEE}
}
```


License
-----

GPMP2 is released under the BSD license, reproduced in [LICENSE](LICENSE).
