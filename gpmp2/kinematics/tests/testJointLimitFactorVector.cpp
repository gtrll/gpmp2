/**
*  @file testJointLimitFactorVector.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpmp2/kinematics/JointLimitFactorVector.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(JointLimitFactorVector, error) {

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(2, 1.0);

  // 2 link simple example
  Vector2 dlimit(-5.0, -10.0), ulimit(5, 10.0);
  Vector2 thresh(2.0, 2.0);
  JointLimitFactorVector factor(0, cost_model, dlimit, ulimit, thresh);
  Vector2 conf;
  Vector actual, expect;
  Matrix H_exp, H_act;

  // zero
  conf = Vector2(0.0, 0.0);
  actual = factor.evaluateError(conf, H_act);
  expect = Vector2(0.0, 0.0);
  H_exp = numericalDerivative11(boost::function<Vector2(const Vector2&)>(
      boost::bind(&JointLimitFactorVector::evaluateError, factor, _1, boost::none)), conf, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));

  // over down limit
  conf = Vector2(-10.0, -10.0);
  actual = factor.evaluateError(conf, H_act);
  expect = Vector2(7.0, 2.0);
  H_exp = numericalDerivative11(boost::function<Vector2(const Vector2&)>(
    boost::bind(&JointLimitFactorVector::evaluateError, factor, _1, boost::none)), conf, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));

  // over up limit
  conf = Vector2(10.0, 10.0);
  actual = factor.evaluateError(conf, H_act);
  expect = Vector2(7.0, 2.0);
  H_exp = numericalDerivative11(boost::function<Vector2(const Vector2&)>(
    boost::bind(&JointLimitFactorVector::evaluateError, factor, _1, boost::none)), conf, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));
}

/* ************************************************************************** */
TEST(JointLimitFactorVector, optimization_1) {
  // zero point

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(2, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(2, 1000);
  Key qkey = Symbol('x', 0);
  Vector2 dlimit(-5.0, -10.0), ulimit(5, 10.0);
  Vector2 thresh(2.0, 2.0);

  Vector conf;
  conf = (Vector(2) << 0.0, 0.0).finished();

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactorVector(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-6);
  EXPECT(assert_equal(conf, results.at<Vector>(qkey), 1e-6));
}

/* ************************************************************************** */
TEST(JointLimitFactorVector, optimization_2) {
  // over down limit

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(2, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(2, 1000);
  Key qkey = Symbol('x', 0);
  Vector2 dlimit(-5.0, -10.0), ulimit(5, 10.0);
  Vector2 thresh(2.0, 2.0);

  Vector conf;
  conf = (Vector(2) << -10.0, -10.0).finished();

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactorVector(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  Vector conf_limit = (Vector(2) << -3.0, -8.0).finished();
  EXPECT(assert_equal(conf_limit, results.at<Vector>(qkey), 1e-6));
}

/* ************************************************************************** */
TEST(JointLimitFactorVector, optimization_3) {
  // over up limit

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(2, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(2, 1000);
  Key qkey = Symbol('x', 0);
  Vector2 dlimit(-5.0, -10.0), ulimit(5, 10.0);
  Vector2 thresh(2.0, 2.0);

  Vector conf;
  conf = (Vector(2) << 10.0, 10.0).finished();

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactorVector(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  Vector conf_limit = (Vector(2) << 3.0, 8.0).finished();
  EXPECT(assert_equal(conf_limit, results.at<Vector>(qkey), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}


