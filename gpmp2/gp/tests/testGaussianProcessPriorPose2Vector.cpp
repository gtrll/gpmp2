/**
*  @file testGaussianProcessPriorPose2Vector.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(GaussianProcessPriorPose2Vector, Factor) {

  const double delta_t = 0.1;
  Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  Key key_pose1 = Symbol('x', 1), key_pose2 = Symbol('x', 2);
  Key key_vel1 = Symbol('v', 1), key_vel2 = Symbol('v', 2);
  GaussianProcessPriorPose2Vector factor(key_pose1, key_vel1, key_pose2, key_vel2, delta_t, Qc_model);
  Pose2Vector p1, p2;
  Vector6 v1, v2;
  Matrix actualH1, actualH2, actualH3, actualH4;
  Matrix expectH1, expectH2, expectH3, expectH4;
  Vector actual, expect;


  // test at origin
  p1 = Pose2Vector(Pose2(0, 0, 0), Vector3(0, 0, 0));
  p2 = Pose2Vector(Pose2(0, 0, 0), Vector3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
  expectH1 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // test at const forward velocity v1 = v2 = 1.0;
  p1 = Pose2Vector(Pose2(0, 0, 0), Vector3(0, 0, 0));
  p2 = Pose2Vector(Pose2(0.1, 0, 0), Vector3(0, 0.2, 0));
  v1 = (Vector6() << 1, 0, 0, 0, 2, 0).finished();
  v2 = (Vector6() << 1, 0, 0, 0, 2, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
  expectH1 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-4));
  EXPECT(assert_equal(expectH2, actualH2, 1e-4));
  EXPECT(assert_equal(expectH3, actualH3, 1e-4));
  EXPECT(assert_equal(expectH4, actualH4, 1e-4));


  // test at const rotation w1 = w2 = 1.0;
  p1 = Pose2Vector(Pose2(0, 0, 0), Vector3(0, 0, 0));
  p2 = Pose2Vector(Pose2(0, 0, 0.1), Vector3(0, 0, 0));
  v1 = (Vector6() << 0, 0, 1, 0, 0, 0).finished();
  v2 = (Vector6() << 0, 0, 1, 0, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(12) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
  expectH1 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));


  // some random stuff just for testing jacobian (error is not zero)
  p1 = Pose2Vector(Pose2(3, -8, 2), Vector3(5, 0.9, 0.7));
  p2 = Pose2Vector(Pose2(-9, 3, 4), Vector3(-6, -0.2, 0.8));
  v1 = (Vector6() << 0.5, 0.9, 0.7, 3, -8, 2).finished();
  v2 = (Vector6() <<0.6, -0.2, 0.8, -9, 3, 4).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expectH1 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivativeDynamic(boost::function<Vector(const Vector6&)>(
      boost::bind(&GaussianProcessPriorPose2Vector::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

}

/* ************************************************************************** */

TEST(GaussianProcessPriorPose2Vector, Optimization) {
  /**
   * A simple graph:
   *
   * p1   p2
   * |    |
   * x1   x2
   *  \  /
   *   gp
   *  /  \
   * v1  v2
   *
   * p1 and p2 are pose prior factor to fix the poses, gp is the GP factor
   * that get correct velocity of v2
   */

  noiseModel::Isotropic::shared_ptr model_prior = noiseModel::Isotropic::Sigma(6, 0.001);
  const double delta_t = 0.1;
  const Matrix Qc = 0.01 * Matrix::Identity(6,6);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  Pose2Vector pose1(Pose2(0, 0, 0), Vector3(0, 0, 0));
  Pose2Vector pose2(Pose2(0.1, 0, 0), Vector3(0, 0.2, 0));
  Vector v1 = (Vector(6) << 1, 0, 0, 0, 2, 0).finished();
  Vector v2 = (Vector(6) << 1.2, 0.3, 0.4, 1.0, 1.0, -1.0).finished();   // rnd value

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose2Vector>(Symbol('x', 1), pose1, model_prior));
  graph.add(PriorFactor<Pose2Vector>(Symbol('x', 2), pose2, model_prior));
  //graph.add(PriorFactor<Vector6>(Symbol('v', 1), v1, model_prior));
  graph.add(GaussianProcessPriorPose2Vector(Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, Qc_model));

  Values init_values;
  init_values.insert(Symbol('x', 1), pose1);
  init_values.insert(Symbol('v', 1), v1);
  init_values.insert(Symbol('x', 2), pose2);
  init_values.insert(Symbol('v', 2), v2);

  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(pose1, values.at<Pose2Vector>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal(pose2, values.at<Pose2Vector>(Symbol('x', 2)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 1)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 2)), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
