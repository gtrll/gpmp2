/**
*  @file    testGaussianPriorWorkspacePose.cpp
*  @author  Mustafa Mukadam
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>
#include <gpmp2/kinematics/ArmModel.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(GaussianPriorWorkspacePoseArm, error) {

  Vector2 a(1, 1), alpha(M_PI/2, 0), d(0, 0);
  ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
  Vector2 q;
  Pose3 des;
  Vector actual, expect;
  Matrix H_exp, H_act;
  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(6, 1.0);

  q = Vector2(M_PI/4.0, -M_PI/2);
  des = Pose3();
  GaussianPriorWorkspacePoseArm factor(0, arm, 1, des, cost_model);
  actual = factor.evaluateError(q, H_act);
  expect = (Vector(6) << 0.613943126, 1.48218982, -0.613943126, 1.1609828, 0.706727485, -0.547039678).finished();
  H_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&GaussianPriorWorkspacePoseArm::evaluateError, factor, _1, boost::none)), q, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));
}


/* ************************************************************************** */
TEST(GaussianPriorWorkspacePoseArm, optimization) {

  noiseModel::Gaussian::shared_ptr cost_model = noiseModel::Isotropic::Sigma(6, 0.1);

  Vector a = (Vector(2) << 1, 1).finished();
  Vector alpha = (Vector(2) << 0, 0).finished();
  Vector d = (Vector(2) << 0, 0).finished();
  ArmModel arm = ArmModel(Arm(2, a, alpha, d), BodySphereVector());
  Pose3 des = Pose3(Rot3(), Point3(2, 0, 0));

  Key qkey = Symbol('x', 0);
  Vector q = (Vector(2) << 0, 0).finished();
  Vector qinit = (Vector(2) << M_PI/2, M_PI/2).finished();

  NonlinearFactorGraph graph;
  graph.add(GaussianPriorWorkspacePoseArm(qkey, arm, 1, des, cost_model));
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
