/**
*  @file testDynamicVector.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gpmp2/geometry/DynamicVector.h>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


GTSAM_CONCEPT_TESTABLE_INST(DynamicVector)
GTSAM_CONCEPT_LIE_INST(DynamicVector)

/* ************************************************************************** */
TEST(DynamicVector, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<DynamicVector>));
  BOOST_CONCEPT_ASSERT((IsManifold<DynamicVector>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<DynamicVector>));
  BOOST_CONCEPT_ASSERT((IsVectorSpace<DynamicVector>));
}

/* ************************************************************************** */
TEST_UNSAFE(DynamicVector, contructor) {
  Vector a(3);
  a << 1,2,3;
  DynamicVector v(a);

  EXPECT_LONGS_EQUAL(3, v.dim());
  EXPECT(assert_equal(a, v.vector(), 1e-9));
}

/* ************************************************************************** */
TEST_UNSAFE(DynamicVector, access_element) {
  Vector a(3);
  a << 1,2,3;
  DynamicVector v(a);

  for (size_t i = 1; i < 3; i++)
    EXPECT_DOUBLES_EQUAL(a(i), v(i), 1e-9);
}

/* ************************************************************************** */
TEST_UNSAFE(DynamicVector, equals) {
  Vector a(3), b(3);
  a << 1,2,3;
  b << 4,5,6;
  DynamicVector v1(a), v2(a), v3(b);

  EXPECT(assert_equal(v1, v2, 1e-9));
  EXPECT(assert_equal(v2, v1, 1e-9));
  EXPECT(!assert_equal(v1, v3, 1e-9));
}

/* ************************************************************************** */
TEST_UNSAFE(DynamicVector, addition) {
  Vector a(3), b(3), c(3);
  a << 1,2,3;
  b << 4,5,6;
  c << 5,7,9;
  DynamicVector v1(a), v2(b), v3(c);

  EXPECT(assert_equal(v3, v1 + v2, 1e-9));
  EXPECT(assert_equal(v3, v2 + v1, 1e-9));
  EXPECT(assert_equal(v3, v1 + b, 1e-9));
}

/* ************************************************************************** */
TEST_UNSAFE(DynamicVector, subtraction) {
  Vector a(3), b(3), c(3);
  a << 1,2,3;
  b << 4,5,6;
  c << 5,7,9;
  DynamicVector v1(a), v2(b), v3(c);

  EXPECT(assert_equal(v1, v3 - v2, 1e-9));
  EXPECT(assert_equal(v2, v3 - v1, 1e-9));
}

/* ************************************************************************** */
TEST_UNSAFE(DynamicVector, optimization) {

  Vector a(3), b(3), c(3);
  a << 1,2,3;
  b << 4,5,6;
  DynamicVector v1(a), v2(b);

  // prior factor graph
  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(3, 0.001);
  NonlinearFactorGraph graph;
  graph.add(PriorFactor<DynamicVector>(Symbol('x', 1), v1, model_prior));

  // init values
  Values init_values;
  init_values.insert(Symbol('x', 1), v2);

  // optimize!
  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(v1, values.at<DynamicVector>(Symbol('x', 1)), 1e-9));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
