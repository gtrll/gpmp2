C++ development guide
=========================================
Although all examples are given in Matlab scripts, all the core APIs are developed in C++, 
and it's easy to develop your C++ examples or use GPMP2 in your C++ projects. 
Here we briefly explain how to use GPMP2 in a C++ project.

Link to your own projects
-----

We provide easy linking to external *CMake* projects. Add the following lines to your CMakeLists.txt

```cmake
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})
```

and link your targets to gpmp2

```cmake
target_link_libraries(${YOUR_BUILD_TARGET} gpmp2)
```

It's also OK to directly add header files and shared library path to your compiler's flags, if you are not using CMake.

C++ source code structure
-----

All the C++ source code is located in [gpmp2](../gpmp2/) folder. The source files are divided in several sub folders, 
based on their functionalities.

- **gp** contains all Gaussian Process related classes and functions that include GP prior factors, and GP interpolation.
- **kinematics** contains all robot model related classes and functions that include forward kinematics models, and sphere-based robot physical models.
- **obstacle** contains all obstacle related classes and functions that include signed distance fields, obstacle cost/likelihood functions, and obstacle avoidance factors.
- **planner** contains all integrated GPMP2 motion planners that include batch planners and iSAM2 replanners.
- **utils** contains some utility functions.

You can explore the header files in sub folders to further learn the interfaces.

Build your own robot arm
-----

If you want to build your own robot arm model in C++, there are two steps needed: 
first is to build an *abstract* robot arm, which contains only forward kinematics information, 
second is to build a *physical* robot arm, which contains forward kinematics information plus physical body information.

To construct an abstract robot arm, call [Arm](../gpmp2/kinematics/Arm.h) class constructor:

```cpp
class Arm : public ForwardKinematics<gtsam::Vector, gtsam::Vector> {
  ...
  /// Contructor take in number of joints for the arm, its DH parameters
  /// the base pose (default zero pose), and theta bias (default zero)
  Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d,
      const gtsam::Pose3& base_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0,0,0)),
      boost::optional<const gtsam::Vector&> theta_bias = boost::none);
  ...
```

The constructor takes degree of freedom, DH parameters (a, \alpha, d, and optional \theta bias), 
and optional arm base pose (default zero), builds an abstract arm object. 

In GPMP2, the physical robot information is stored by a series of spheres, in [BodySphereVector](../gpmp2/kinematics/RobotModel.h) class. 

```cpp
/// vector of body sphere, typedef here to wrap in matlab
typedef std::vector<BodySphere> BodySphereVector;
```

Each sphere in [BodySphere](../gpmp2/kinematics/RobotModel.h) needs a radius, attached 
link index (0 - nr_link-1), and relative position to the link.

```cpp
/// body sphere class, each one indicate a body sphere
struct BodySphere {
  size_t link_id;       // attched link id, 0 - nr_link-1
  double radius;        // sphere radius
  gtsam::Point3 center;        // sphere center position to the link base
  // constructor
  BodySphere(size_t id, double r, const gtsam::Point3& c) : link_id(id), radius(r), center(c) {}
  ...
```
Once you initialize the sphere information in [BodySphereVector](../gpmp2/kinematics/RobotModel.h),
you can construct an [ArmModel](../gpmp2/kinematics/ArmModel.h) class, which is a templated version of 
[RobotModel](../gpmp2/kinematics/RobotModel.h), with the forward kinematics [Arm](../gpmp2/kinematics/Arm.h)
and physical model information [BodySphereVector](../gpmp2/kinematics/RobotModel.h).

Matlab toolbox wrapping
-----

All the code APIs should be first developed in C++, and then wrapped in Matlab.
All the C++ classes and functions need to be wrapped in Matlab and are declared in [gpmp2.h](../gpmp2.h) header,
and GPMP2 uses GTSAM matlab wrapper to automatically generate .mex library and .m files for all C++ interfaces wrapped.

If you would like to wrap your own C++ interfaces (classes and functions) in Matlab,
just put the declarations in [gpmp2.h](../gpmp2.h) header file, and to import the Matlab toolbox
add the following line at the beginning of Matlab script before using it.

```matlab
import gpmp2.*
```