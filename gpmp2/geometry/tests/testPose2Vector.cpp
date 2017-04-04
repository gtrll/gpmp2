/**
*  @file testPose2Vector.cpp
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

#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(Pose2Vector, Lie) {
  BOOST_CONCEPT_ASSERT((IsGroup<Pose2Vector>));
  BOOST_CONCEPT_ASSERT((IsManifold<Pose2Vector>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Pose2Vector>));
}

/* ************************************************************************** */
TEST(Pose2Vector, Contructors) {
  Pose2Vector p1;     // default nothing
  Pose2Vector pi(Pose2(), Vector::Zero(3));     // manual identity
  p1 = pi;  // assignment

  Vector d(6);
  d << 0.1, 0.2, 0.3, 4, 5, 6;
  Pose2Vector expected(Pose2::Expmap((Vector(3) << 0.1, 0.2, 0.3).finished()),
      (Vector(3) << 4, 5, 6).finished());
  Pose2Vector p2 = p1.expmap(d);
  EXPECT(assert_equal(expected, p2, 1e-9));
  EXPECT(assert_equal(d, p1.logmap(p2), 1e-9));
}

/* ************************************************************************** */
TEST(Pose2Vector, access) {
  Pose2 exppose = Pose2::Expmap((Vector(3) << 0.1, 0.2, 0.3).finished());
  Vector expvec = (Vector(3) << 4, 5, 6).finished();
  Pose2Vector pv(exppose, expvec);
  EXPECT(assert_equal(exppose, pv.pose(), 1e-9));
  EXPECT(assert_equal(expvec, pv.configuration(), 1e-9));
}

/* ************************************************************************** */
Pose2Vector compose_proxy(const Pose2Vector& A, const Pose2Vector& B) {
  return A.compose(B);
}
TEST(Pose2Vector, compose) {
  Pose2Vector state1(Pose2(1, 1, M_PI_2), Vector3(0.2, -0.1, 0.3)), state2 = state1;
  Pose2Vector expcomp(Pose2(0, 2, M_PI), Vector3(0.4, -0.2, 0.6)), actcomp;

  Matrix actH1, actH2;
  actcomp = state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivativeDynamic<Pose2Vector, Pose2Vector>(
      boost::bind(compose_proxy, _1, state2), state1);
  Matrix numericH2 = numericalDerivativeDynamic<Pose2Vector, Pose2Vector>(
      boost::bind(compose_proxy, state1, _1), state2);

  EXPECT(assert_equal(expcomp, actcomp, 1e-9));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

/* ************************************************************************** */
Pose2Vector between_proxy(const Pose2Vector& A, const Pose2Vector& B) {
  return A.between(B);
}
TEST(Pose2Vector, between) {
  Pose2Vector state1(Pose2(1, 1, M_PI_2), Vector3(0.2, -0.1, 0.3));
  Pose2Vector state2(Pose2(0, 2, M_PI), Vector3(0.5, 0.3, 0.1));
  Pose2Vector expbtw(Pose2(1, 1, M_PI_2), Vector3(0.3, 0.4, -0.2)), actbtw;

  Matrix actH1, actH2;
  actbtw = state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivativeDynamic<Pose2Vector, Pose2Vector>(
      boost::bind(between_proxy, _1, state2), state1);
  Matrix numericH2 = numericalDerivativeDynamic<Pose2Vector, Pose2Vector>(
      boost::bind(between_proxy, state1, _1), state2);

  EXPECT(assert_equal(expbtw, actbtw, 1e-9));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

/* ************************************************************************** */
Pose2Vector inverse_proxy(const Pose2Vector& A) {
  return A.inverse();
}
TEST(Pose2Vector, inverse) {
  Pose2Vector state1(Pose2(1, 1, M_PI_2), Vector3(0.2, -0.1, 0.3));
  Pose2Vector expinv(Pose2(-1, 1, -M_PI_2), Vector3(-0.2, 0.1, -0.3)), actinv;

  Matrix actH1;
  actinv = state1.inverse(actH1);
  Matrix numericH1 = numericalDerivativeDynamic<Pose2Vector, Pose2Vector>(
      boost::bind(inverse_proxy, _1), state1);

  EXPECT(assert_equal(expinv, actinv, 1e-9));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
}

/* ************************************************************************* */
TEST(Pose2Vector, optimization) {
  Pose2Vector state1(Pose2(3, 4, 5), Vector3(3,4,2)), state2(Pose2(1, 5, 7), Vector3(2,3,4));

  // prior factor graph
  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(6, 0.001);
  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Pose2Vector>(Symbol('x', 1), state1, model_prior));

  // init values
  Values init_values;
  init_values.insert(Symbol('x', 1), state2);

  // optimize!
  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(state1, values.at<Pose2Vector>(Symbol('x', 1)), 1e-9));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
