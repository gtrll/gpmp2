GPMP2
===================================================
This library is an implementation of GPMP2 (Gaussian Process Motion Planner 2) algorithm described in [Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs](http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf) (RSS 2016). The core library is developed in C++ language, and an optional Matlab toolbox is provided. Examples are provided in Matlab scripts. A ROS interface is also available within [PIPER](https://github.com/gtrll/piper).

GPMP2 was developed by [Jing Dong](mailto:thu.dongjing@gmail.com) and 
[Mustafa Mukadam](http://www.mustafamukadam.com) as part of their work at Georgia Tech Robot Learning Lab. 

Prerequisites
------

- CMake >= 2.6 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.50 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://github.com/borglab/gtsam) >= 4.0 alpha, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision.
Here we use factor graph implementations and inference/optimization tools provided by GTSAM.

Compilation & Installation
------

In the library folder execute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make check  # optional, run unit tests
$ make install
```

Matlab Toolbox
-----

An optional Matlab toolbox is provided to use our library in Matlab. To enable Matlab toolbox during compilation:

```
$ cmake -DGPMP2_BUILD_MATLAB_TOOLBOX:OPTION=ON -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=/path/install/toolbox ..
$ make install
```

After you install the Matlab toolbox, don't forget to add `/path/install/toolbox` to your Matlab path.

Tested Compatibility
-----

The gpmp2 library is designed to be cross-platform. It has been tested on Ubuntu Linux and Windows for now.

- Ubuntu: GCC 4.8 - 4.9, 5.3 - 5.4
- Windows: Visual C++ 2015 (Matlab toolbox not tested)
- Boost: 1.50 - 1.61


Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Jing Dong](mailto:thu.dongjing@gmail.com)
or [Mustafa Mukadam](mailto:mmukadam3@gatech.edu) .


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

GPMP2 is released under the BSD license, reproduced in the file LICENSE in this directory.
