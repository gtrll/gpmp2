/**
*  @file testGaussianProcessInterpolatorLinear.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


// only test the 3 dim linear case (for Pose2 approximation)
typedef GaussianProcessInterpolatorLinear GPBase;

/* ************************************************************************** */
TEST(GaussianProcessInterpolatorLinear, equals) {
  Matrix3 Qc = 0.01 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.03;
  GPBase base1(Qc_model, dt, tau), base2(Qc_model, dt, tau);
  EXPECT(assert_equal(base1, base2, 1e-9));

  Matrix3 Qc2 = 0.02 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc2_model = noiseModel::Gaussian::Covariance(Qc2);
  GPBase base3(Qc2_model, dt, tau);
  EXPECT(!assert_equal(base1, base3, 1e-9));

  dt = 0.2, tau = 0.03;
  GPBase base4(Qc_model, dt, tau);
  EXPECT(!assert_equal(base1, base4, 1e-9));

  dt = 0.1, tau = 0.06;
  GPBase base5(Qc_model, dt, tau);
  EXPECT(!assert_equal(base1, base5, 1e-9));
}

/* ************************************************************************** */
TEST(GaussianProcessInterpolatorLinear, interpolatePose) {
  Vector3 p1, p2, expect, actual;
  Vector3 v1, v2;
  Matrix actualH1, actualH2, actualH3, actualH4;
  Matrix expectH1, expectH2, expectH3, expectH4;
  Matrix Qc = 0.01 * Matrix::Identity(3,3);
  noiseModel::Gaussian::shared_ptr Qc_model = noiseModel::Gaussian::Covariance(Qc);
  double dt = 0.1, tau = 0.03;
  GPBase base(Qc_model, dt, tau);

  // test at origin
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 0, 0, 0).finished();
  v1 = (Vector3() << 0, 0, 0).finished();
  v2 = (Vector3() << 0, 0, 0).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = (Vector3() << 0, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

  // test forward velocity
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 1, 0, 0).finished();
  v1 = (Vector3() << 10, 0, 0).finished();
  v2 = (Vector3() << 10, 0, 0).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = (Vector3() << 0.3, 0, 0).finished();
  expectH1 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));

  // test rotate
  p1 = (Vector3() << 0, 0, 0).finished();
  p2 = (Vector3() << 0, 0, 0.3).finished();
  v1 = (Vector3() << 0, 0, 3).finished();
  v2 = (Vector3() << 0, 0, 3).finished();
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expect = (Vector3() << 0, 0, 0.09).finished();
  expectH1 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
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
  actual = base.interpolatePose(p1, v1, p2, v2, actualH1, actualH2,
      actualH3, actualH4);
  expectH1 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          _1, v1, p2, v2, boost::none, boost::none, boost::none, boost::none)), p1, 1e-6);
  expectH2 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, _1, p2, v2, boost::none, boost::none, boost::none, boost::none)), v1, 1e-6);
  expectH3 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, _1, v2, boost::none, boost::none, boost::none, boost::none)), p2, 1e-6);
  expectH4 = numericalDerivative11(boost::function<Vector3(const Vector3&)>(
      boost::bind(&GPBase::interpolatePose, base,
          p1, v1, p2, _1, boost::none, boost::none, boost::none, boost::none)), v2, 1e-6);
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
