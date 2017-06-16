GPMP2
===================================================
This library is an implementation of GPMP2 (Gaussian Process Motion Planner 2) algorithm described in [Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs](http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf) (RSS 2016). The core library is developed in C++ language, and an optional Matlab toolbox is provided. Examples are provided in Matlab scripts. An OpenRAVE plugin, [orgpmp2](https://github.com/gtrll/gpmp2_orplugin), is also available with examples.

GPMP2 is being developed by [Jing Dong](mailto:thu.dongjing@gmail.com) and 
[Mustafa Mukadam](mailto:mmukadam3@gatech.edu) as part of their work at Georgia Tech Robot Learning Lab. 

Prerequisites
------

- CMake >= 2.6 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.50 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) >= 4.0 alpha, a C++ library that implement smoothing and mapping (SAM) framework in robotics and vision.
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
@inproceedings{Dong-RSS-16,
  Author = "Jing Dong and Mustafa Mukadam and Frank Dellaert and Byron Boots",
  booktitle = {Proceedings of Robotics: Science and Systems (RSS-2016)},
  Title = "Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs",
  year = {2016}
}

@article{Dong17arxiv,
  author    = {Jing Dong and Byron Boots and Frank Dellaert},
  title     = {Sparse Gaussian Processes for Continuous-Time Trajectory Estimation on Matrix Lie Groups},
  journal   = {Arxiv},
  volume    = {abs/1705.06020},
  year      = {2017},
  url       = {http://arxiv.org/abs/1705.06020}
}
```


License
-----

GPMP2 is released under the BSD license, reproduced in the file LICENSE in this directory.
