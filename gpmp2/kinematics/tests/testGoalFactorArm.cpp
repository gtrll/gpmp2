/**
*  @file testGoalFactorArm.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpmp2/kinematics/GoalFactorArm.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(GoalFactorArm, error) {

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(3, 1.0);

  // 2 link simple example
  Vector2 a(1, 1), alpha(0, 0), d(0, 0);
  Arm arm(2, a, alpha, d);
  Vector2 q;
  Point3 goal;
  GoalFactorArm factor;
  Vector actual, expect;
  Matrix H_exp, H_act;

  // zero
  q = Vector2(0, 0);
  goal = Point3(2, 0, 0);
  factor = GoalFactorArm(0, cost_model, arm, goal);
  actual = factor.evaluateError(q, H_act);
  expect = Vector3(0, 0, 0);
  H_exp = numericalDerivative11(boost::function<Vector3(const Vector2&)>(
      boost::bind(&GoalFactorArm::evaluateError, factor, _1, boost::none)), q, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));

  // 45 deg
  q = Vector2(M_PI/4.0, 0);
  goal = Point3(1.414213562373095, 1.414213562373095, 0);
  factor = GoalFactorArm(0, cost_model, arm, goal);
  actual = factor.evaluateError(q, H_act);
  expect = Vector3(0, 0, 0);
  H_exp = numericalDerivative11(boost::function<Vector3(const Vector2&)>(
      boost::bind(&GoalFactorArm::evaluateError, factor, _1, boost::none)), q, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));

  // non zero error
  q = Vector2(M_PI/4.0, 0);
  goal = Point3(2, 0, 0);
  factor = GoalFactorArm(0, cost_model, arm, goal);
  actual = factor.evaluateError(q, H_act);
  expect = Vector3(-0.585786437626905, 1.414213562373095, 0);
  H_exp = numericalDerivative11(boost::function<Vector3(const Vector2&)>(
      boost::bind(&GoalFactorArm::evaluateError, factor, _1, boost::none)), q, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));
}


/* ************************************************************************** */
TEST(GoalFactorArm, optimization_1) {

  // use optimization to solve inverse kinematics
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(3, 0.1);

  // 2 link simple example
  Vector a = (Vector(2) << 1, 1).finished();
  Vector alpha = (Vector(2) << 0, 0).finished();
  Vector d = (Vector(2) << 0, 0).finished();
  Arm arm(2, a, alpha, d);
  Point3 goal(1.414213562373095, 1.414213562373095, 0);

  Key qkey = Symbol('x', 0);
  Vector q = (Vector(2) << M_PI/4.0, 0).finished();
  Vector qinit = (Vector(2) << 0, 0).finished();

  NonlinearFactorGraph graph;
  graph.add(GoalFactorArm(qkey, cost_model, arm, goal));
  Values init_values;
  init_values.insert(qkey, qinit);

  LevenbergMarquardtParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-3);
  EXPECT(assert_equal(q, results.at<Vector>(qkey), 1e-3));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}


