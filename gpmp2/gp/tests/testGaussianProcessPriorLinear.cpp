/**
*  @file testGaussianProcessPriorLinear.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpmp2/gp/GaussianProcessPriorLinear.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


typedef GaussianProcessPriorLinear GPPrior;

/* ************************************************************************** */
TEST_UNSAFE(GaussianProcessPriorLinear, Factor) {

  const double delta_t = 0.1;
  Matrix Qc = 0.01 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  Key key_pose1 = Symbol('x', 1), key_pose2 = Symbol('x', 2);
  Key key_vel1 = Symbol('v', 1), key_vel2 = Symbol('v', 2);
  GPPrior factor(key_pose1, key_vel1, key_pose2, key_vel2, delta_t, Qc_model);
  Vector3 p1, p2;
  Vector3 v1, v2;
  Matrix actualH1, actualH2, actualH3, actualH4;
  Matrix expectH1, expectH2, expectH3, expectH4;
  Vector actual, expect;

  // test at origin
  p1 = (Vector(3) << 0, 0, 0).finished();
  p2 = (Vector(3) << 0, 0, 0).finished();
  v1 = (Vector(3) << 0, 0, 0).finished();
  v2 = (Vector(3) << 0, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

  // test const forward v1 = v2 = 1.0
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 0.1, 0, 0).finished();
  v1 = (Vector3() << 1, 0, 0).finished();
  v2 = (Vector3() << 1, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

  // test const rotation w = 1.0
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 0, 0, 0.1).finished();
  v1 = (Vector3() << 0, 0, 1).finished();
  v2 = (Vector3() << 0, 0, 1).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expect = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

  // some random stuff just for testing jacobian (error is not zero)
  p1 = (Vector3() << 2, -5, 7).finished();
  p2 = (Vector3() << -8, 4, -8).finished();
  v1 = (Vector3() << -1, 2, -9).finished();
  v2 = (Vector3() << 3, -4, 7).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, actualH1, actualH2, actualH3, actualH4);
  expectH1 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector(const Vector3&)>(
      boost::bind(&GPPrior::evaluateError, factor,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

}

/* ************************************************************************** */
TEST(GaussianProcessPriorLinear, Optimization) {

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
   * that get correct velocity of v1 and v2
   */

  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(3, 0.001);
  double delta_t = 0.1;
  Matrix Qc = 0.01 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);

  Vector p1 = (Vector(3) << 1, 0, 0).finished();
  Vector p2 = (Vector(3) << 1.1, 0, 0).finished();
  Vector v1 = (Vector(3) << 1, 0, 0).finished();
  Vector v2 = (Vector(3) << 1, 0, 0).finished();

  // noisy init values
  Vector p1init = (Vector(3) << 1.1, 0.3, 0.5).finished();
  Vector p2init = (Vector(3) << 1.2, 0.4, -0.2).finished();
  Vector v1init = (Vector(3) << 1, 0.1, 0.2).finished();
  Vector v2init = (Vector(3) << 2.1, -1.2, 0.9).finished();

  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Vector>(Symbol('x', 1), p1, model_prior));
  graph.add(PriorFactor<Vector>(Symbol('x', 2), p2, model_prior));
  //graph.add(PriorFactor<Vector3>(Symbol('v', 1), v1, model_prior));

  graph.add(GPPrior(Symbol('x', 1), Symbol('v', 1),
      Symbol('x', 2), Symbol('v', 2), delta_t, Qc_model));

  Values init_values;
  init_values.insert(Symbol('x', 1), p1init);
  init_values.insert(Symbol('v', 1), v1init);
  init_values.insert(Symbol('x', 2), p2init);
  init_values.insert(Symbol('v', 2), v2init);

  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(p1, values.at<Vector>(Symbol('x', 1)), 1e-6));
  EXPECT(assert_equal(p2, values.at<Vector>(Symbol('x', 2)), 1e-6));
  EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 1)), 1e-6));
  EXPECT(assert_equal(v2, values.at<Vector>(Symbol('v', 2)), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
