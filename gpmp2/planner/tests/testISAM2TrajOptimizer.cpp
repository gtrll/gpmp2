/**
*  @file testISAM2TrajOptimizer.cpp
*  @author Jing Dong
*  @date May 30, 2016
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/planner/ISAM2TrajOptimizer.h>

#include <iostream>


using namespace std;
using namespace gtsam;
using namespace gpmp2;



/* ************************************************************************** */
TEST(ISAM2TrajOptimizer, constructor) {

  // 2 link simple example, with none zero base poses
  Vector2 a(1, 1), alpha(0, 0), d(0, 0);
  Pose3 base_pose(Rot3(), Point3(2.0, 1.0, -1.0));
  Arm abs_arm(2, a, alpha, d, base_pose);
  // body spheres
  BodySphereVector body_spheres;
  body_spheres.push_back(BodySphere(0, 0.5, Point3(-1.0, 0, 0)));
  body_spheres.push_back(BodySphere(0, 0.1, Point3(-0.5, 0, 0)));
  body_spheres.push_back(BodySphere(0, 0.1, Point3(0, 0, 0)));
  body_spheres.push_back(BodySphere(1, 0.1, Point3(-0.5, 0, 0)));
  body_spheres.push_back(BodySphere(1, 0.1, Point3(0, 0, 0)));
  ArmModel arm(abs_arm, body_spheres);

  // sdf
  vector<Matrix> data;
  data.push_back((Matrix(5, 5) <<
      1.7321, 1.4142, 1.4142, 1.4142, 1.7321,
      1.4142, 1, 1, 1, 1.4142,
      1.4142, 1, 1, 1, 1.4142,
      1.4142, 1, 1, 1, 1.4142,
      1.7321, 1.4142, 1.4142, 1.4142, 1.7321).finished());
  data.push_back((Matrix(5, 5) <<
      1.4142, 1, 1, 1, 1.4142,
      1, 0, 0, 0, 1,
      1, 0, 0, 0, 1,
      1, 0, 0, 0, 1,
      1.4142, 1, 1, 1,4142).finished());
  data.push_back((Matrix(5, 5) <<
      1.7321, 1.4142, 1.4142, 1.4142, 1.7321,
      1.4142, 1, 1, 1, 1.4142,
      1.4142, 1, 1, 1, 1.4142,
      1.4142, 1, 1, 1, 1.4142,
      1.7321, 1.4142, 1.4142, 1.4142, 1.7321).finished());
  Point3 origin(-0.2, -0.2, -0.1);
  double cell_size = 0.1;
  SignedDistanceField field(origin, cell_size, data);

  // settings
  TrajOptimizerSetting setting(2);

  ISAM2TrajOptimizer3DArm isam(arm, field, setting);
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
