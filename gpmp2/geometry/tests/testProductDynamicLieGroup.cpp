/**
 * @file testProductDynamicLieGroup.cpp
 * @date May, 2015
 * @author Frank Dellaert
 * @brief unit tests for Lie group type machinery
 */

#include <gpmp2/geometry/ProductDynamicLieGroup.h>
#include <gpmp2/geometry/DynamicLieTraits.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/testLie.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

static const double tol = 1e-9;

//******************************************************************************

// all fix size example used for test
namespace gpmp2 {
typedef ProductDynamicLieGroup<Point2, Pose2> Product;
}

namespace gtsam {
template<> struct traits<Product> : internal::DynamicLieGroupTraits<Product> {
  static void Print(const Product& m, const string& s = "") {
    cout << s << "(" << m.first << "," << m.second.translation() << "/"
        << m.second.theta() << ")" << endl;
  }
  static bool Equals(const Product& m1, const Product& m2, double tol = 1e-8) {
    return traits<Point2>::Equals(m1.first, m2.first, tol) && m1.second.equals(m2.second, tol);
  }
};
}

//******************************************************************************
TEST(ProductDynamicLieGroup, ProductLieGroup) {
  BOOST_CONCEPT_ASSERT((IsGroup<Product>));
  BOOST_CONCEPT_ASSERT((IsManifold<Product>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<Product>));
  Product pair1;
  Vector5 d;
  d << 1, 2, 0.1, 0.2, 0.3;
  Product expected(Point2(1, 2), Pose2::Expmap(Vector3(0.1, 0.2, 0.3)));
  Product pair2 = pair1.expmap(d);
  EXPECT(assert_equal(expected, pair2, 1e-9));
  EXPECT(assert_equal(d, pair1.logmap(pair2), 1e-9));
}

/* ************************************************************************* */
Product compose_proxy(const Product& A, const Product& B) {
  return A.compose(B);
}
TEST( ProductDynamicLieGroup, compose ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2 = state1;

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivativeDynamic<Product, Product>(
      boost::bind(compose_proxy, _1, state2), state1);
  Matrix numericH2 = numericalDerivativeDynamic<Product, Product>(
      boost::bind(compose_proxy, state1, _1), state2);
  EXPECT(assert_equal(numericH1, actH1, tol));
  EXPECT(assert_equal(numericH2, actH2, tol));
}

/* ************************************************************************* */
Product between_proxy(const Product& A, const Product& B) {
  return A.between(B);
}
TEST( ProductDynamicLieGroup, between ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2 = state1;

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivativeDynamic<Product, Product>(
      boost::bind(between_proxy, _1, state2), state1);
  Matrix numericH2 = numericalDerivativeDynamic<Product, Product>(
      boost::bind(between_proxy, state1, _1), state2);
  EXPECT(assert_equal(numericH1, actH1, tol));
  EXPECT(assert_equal(numericH2, actH2, tol));
}

/* ************************************************************************* */
Product inverse_proxy(const Product& A) {
  return A.inverse();
}
TEST( ProductDynamicLieGroup, inverse ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5));

  Matrix actH1;
  state1.inverse(actH1);
  Matrix numericH1 = numericalDerivativeDynamic<Product, Product>(
      boost::bind(inverse_proxy, _1), state1);
  EXPECT(assert_equal(numericH1, actH1, tol));
}

/* ************************************************************************* */
Product expmap_proxy(const Vector5& vec) {
  return Product::Expmap(vec);
}
TEST( ProductDynamicLieGroup, Expmap ) {
  Vector5 vec;
  vec << 1, 2, 0.1, 0.2, 0.3;

  Matrix actH;
  Product::Expmap(vec, actH);
  Matrix numericH = numericalDerivativeDynamic<Product, Vector5>(
      boost::bind(expmap_proxy, _1), vec);
  EXPECT(assert_equal(numericH, actH, tol));
}

/* ************************************************************************* */
Vector5 logmap_proxy(const Product& p) {
  return Product::Logmap(p);
}
TEST( ProductDynamicLieGroup, Logmap ) {
  Product state(Point2(1, 2), Pose2(3, 4, 5));

  Matrix actH;
  Product::Logmap(state, actH);
  Matrix numericH = numericalDerivativeDynamic<Vector5, Product>(
      boost::bind(logmap_proxy, _1), state);
  EXPECT(assert_equal(numericH, actH, tol));
}

/* ************************************************************************* */
TEST( ProductDynamicLieGroup, optimization ) {
  Product state1(Point2(1, 2), Pose2(3, 4, 5)), state2(Point2(3, 4), Pose2(1, 5, 7));

  // prior factor graph
  noiseModel::Isotropic::shared_ptr model_prior =
      noiseModel::Isotropic::Sigma(5, 0.001);
  NonlinearFactorGraph graph;
  graph.add(PriorFactor<Product>(Symbol('x', 1), state1, model_prior));

  // init values
  Values init_values;
  init_values.insert(Symbol('x', 1), state2);

  // optimize!
  GaussNewtonParams parameters;
  GaussNewtonOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values values = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
  EXPECT(assert_equal(state1, values.at<Product>(Symbol('x', 1)), 1e-9));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

