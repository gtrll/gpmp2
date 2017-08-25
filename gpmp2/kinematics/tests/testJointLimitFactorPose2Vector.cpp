/**
*  @file testJointLimitFactorPose2Vector.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gpmp2/kinematics/JointLimitFactorPose2Vector.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(JointLimitFactorPose2Vector, error) {

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(5, 1.0);

  // 2 link simple example
  Vector5 dlimit = (Vector5() << 0, 0, 0, -5.0, -10.0).finished();
  Vector5 ulimit = (Vector5() << 0, 0, 0, 5, 10.0).finished();
  Vector5 thresh = (Vector5() << 0, 0, 0, 2.0, 2.0).finished();
  JointLimitFactorPose2Vector factor(0, cost_model, dlimit, ulimit, thresh);
  Pose2Vector conf;
  Vector actual, expect;
  Matrix H_exp, H_act;

  // zero
  conf = Pose2Vector(Pose2(), Vector2(0.0, 0.0));
  actual = factor.evaluateError(conf, H_act);
  expect = (Vector5() << 0, 0, 0, 0.0, 0.0).finished();
  H_exp = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
      boost::bind(&JointLimitFactorPose2Vector::evaluateError, factor, _1, boost::none)), conf, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));

  // over down limit
  conf = Pose2Vector(Pose2(), Vector2(-10.0, -10.0));
  actual = factor.evaluateError(conf, H_act);
  expect = (Vector5() << 0, 0, 0, 7.0, 2.0).finished();
  H_exp = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
    boost::bind(&JointLimitFactorPose2Vector::evaluateError, factor, _1, boost::none)), conf, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));

  // over up limit
  conf = Pose2Vector(Pose2(), Vector2(10.0, 10.0));
  actual = factor.evaluateError(conf, H_act);
  expect = (Vector5() << 0, 0, 0, 7.0, 2.0).finished();
  H_exp = numericalDerivativeDynamic(boost::function<Vector(const Pose2Vector&)>(
    boost::bind(&JointLimitFactorPose2Vector::evaluateError, factor, _1, boost::none)), conf, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));
}

/* ************************************************************************** */
TEST(JointLimitFactorPose2Vector, optimization_1) {
  // zero point

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(5, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(5, 1000);
  Key qkey = Symbol('x', 0);
  Vector5 dlimit = (Vector5() << 0, 0, 0, -5.0, -10.0).finished();
  Vector5 ulimit = (Vector5() << 0, 0, 0, 5, 10.0).finished();
  Vector5 thresh = (Vector5() << 0, 0, 0, 2.0, 2.0).finished();

  Pose2Vector conf;
  conf = Pose2Vector(Pose2(), Vector2(0, 0));

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactorPose2Vector(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Pose2Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-6);
  EXPECT(assert_equal(conf, results.at<Pose2Vector>(qkey), 1e-6));
}

/* ************************************************************************** */
TEST(JointLimitFactorPose2Vector, optimization_2) {
  // over down limit

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(5, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(5, 1000);
  Key qkey = Symbol('x', 0);
  Vector5 dlimit = (Vector5() << 0, 0, 0, -5.0, -10.0).finished();
  Vector5 ulimit = (Vector5() << 0, 0, 0, 5, 10.0).finished();
  Vector5 thresh = (Vector5() << 0, 0, 0, 2.0, 2.0).finished();

  Pose2Vector conf(Pose2(1, -2, 3), Vector2(-10.0, -10.0));

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactorPose2Vector(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Pose2Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  Vector conf_limit = (Vector(2) << -3.0, -8.0).finished();
  EXPECT(assert_equal(conf_limit, results.at<Pose2Vector>(qkey).configuration(), 1e-6));
}

/* ************************************************************************** */
TEST(JointLimitFactorPose2Vector, optimization_3) {
  // over up limit

  // settings
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(5, 0.001);
  noiseModel::Gaussian::shared_ptr prior_model = noiseModel::Isotropic::Sigma(5, 1000);
  Key qkey = Symbol('x', 0);
  Vector5 dlimit = (Vector5() << 0, 0, 0, -5.0, -10.0).finished();
  Vector5 ulimit = (Vector5() << 0, 0, 0, 5, 10.0).finished();
  Vector5 thresh = (Vector5() << 0, 0, 0, 2.0, 2.0).finished();

  Pose2Vector conf(Pose2(1, -2, 3), Vector2(10.0, 10.0));

  NonlinearFactorGraph graph;
  graph.add(JointLimitFactorPose2Vector(qkey, cost_model, dlimit, ulimit, thresh));
  graph.add(PriorFactor<Pose2Vector>(qkey, conf, prior_model));
  Values init_values;
  init_values.insert(qkey, conf);

  GaussNewtonParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  Vector conf_limit = (Vector(2) << 3.0, 8.0).finished();
  EXPECT(assert_equal(conf_limit, results.at<Pose2Vector>(qkey).configuration(), 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}


