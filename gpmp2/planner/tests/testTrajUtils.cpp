/**
*  @file testTrajUtils.cpp
*  @author Jing Dong
*  @date May 30, 2016
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/geometry/Pose2Vector.h>

#include <iostream>


using namespace std;
using namespace gtsam;
using namespace gpmp2;



/* ************************************************************************** */
TEST(TrajUtils, interpolateLinear) {

  Matrix Qc = 0.01 * Matrix::Identity(2,2);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  // values to be interpolated
  Values init_values;
  // const velocity vb = 10, dt = 0.1
  init_values.insert(Symbol('x', 0), (Vector(2) << 0, 0).finished());
  init_values.insert(Symbol('x', 1), (Vector(2) << 1, 0).finished());
  init_values.insert(Symbol('v', 0), (Vector(2) << 10, 0).finished());
  init_values.insert(Symbol('v', 1), (Vector(2) << 10, 0).finished());

  // interpolate 5 interval in the middle
  Values inter_values = interpolateArmTraj(init_values, Qc_model, 0.1, 4);

  EXPECT(assert_equal((Vector(2) << 0, 0).finished(), inter_values.at<Vector>(Symbol('x', 0)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 0.2, 0).finished(), inter_values.at<Vector>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 0.4, 0).finished(), inter_values.at<Vector>(Symbol('x', 2)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 0.6, 0).finished(), inter_values.at<Vector>(Symbol('x', 3)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 0.8, 0).finished(), inter_values.at<Vector>(Symbol('x', 4)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 1, 0).finished(), inter_values.at<Vector>(Symbol('x', 5)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 10, 0).finished(), inter_values.at<Vector>(Symbol('v', 0)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 10, 0).finished(), inter_values.at<Vector>(Symbol('v', 1)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 10, 0).finished(), inter_values.at<Vector>(Symbol('v', 2)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 10, 0).finished(), inter_values.at<Vector>(Symbol('v', 3)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 10, 0).finished(), inter_values.at<Vector>(Symbol('v', 4)), 1e-6));
  EXPECT(assert_equal((Vector(2) << 10, 0).finished(), inter_values.at<Vector>(Symbol('v', 5)), 1e-6));
}

/* ************************************************************************** */
TEST(TrajUtils, initPose2VectorTrajStraightLine_1) {
  Values init_values = initPose2VectorTrajStraightLine(
      Pose2(1,3,M_PI-0.5), (Vector(2) << 2,4).finished(), 
      Pose2(3,7,-M_PI+0.5), (Vector(2) << 4,8).finished(), 5);

  EXPECT(assert_equal(Pose2Vector(Pose2(1,3,M_PI-0.5), (Vector(2) << 2,4).finished()), 
      init_values.at<Pose2Vector>(Symbol('x', 0)), 1e-6));
  //EXPECT(assert_equal(Pose2Vector(Pose2(1.4,3.8,M_PI-0.3), (Vector(2) << 2.4,4.8).finished()), 
  //    init_values.at<Pose2Vector>(Symbol('x', 1)), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
