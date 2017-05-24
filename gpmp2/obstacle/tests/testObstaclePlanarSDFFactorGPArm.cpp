/**
*  @file ObstaclePlanarSDFFactorGPArm.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


inline Vector errorWrapper(const ObstaclePlanarSDFFactorGPArm& factor,
    const Vector& conf1, const Vector& vel1,
    const Vector& conf2, const Vector& vel2) {
  return factor.evaluateError(conf1, vel1, conf2, vel2);
}


// convert sdf vector to hinge loss err vector
inline Vector convertSDFtoErr(const Vector& sdf, double eps) {
  Vector err_ori = 0.0 - sdf.array() + eps;
  return (err_ori.array() > 0.0).select(err_ori, Vector::Zero(err_ori.rows()));  // (R < s ? P : Q)
}

/* ************************************************************************** */
// signed distance field data
Matrix field2, map_ground_truth2;
PlanarSDF sdf2;

TEST(ObstaclePlanarSDFFactorGPArm, data) {

  map_ground_truth2 = (Matrix(7, 7) <<
      0,     0,     0,     0,     0,     0,     0,
      0,     0,     0,     0,     0,     0,     0,
      0,     0,     1,     1,     1,     0,     0,
      0,     0,     1,     1,     1,     0,     0,
      0,     0,     1,     1,     1,     0,     0,
      0,     0,     0,     0,     0,     0,     0,
      0,     0,     0,     0,     0,     0,     0).finished();
  field2 = (Matrix(7, 7) <<
      2.8284,    2.2361,    2.0000,    2.0000,    2.0000,    2.2361,    2.8284,
      2.2361,    1.4142,    1.0000,    1.0000,    1.0000,    1.4142,    2.2361,
      2.0000,    1.0000,   -1.0000,   -1.0000,   -1.0000,    1.0000,    2.0000,
      2.0000,    1.0000,   -1.0000,   -2.0000,   -1.0000,    1.0000,    2.0000,
      2.0000,    1.0000,   -1.0000,   -1.0000,   -1.0000,    1.0000,    2.0000,
      2.2361,    1.4142,    1.0000,    1.0000,    1.0000,    1.4142,    2.2361,
      2.8284,    2.2361,    2.0000,    2.0000,    2.0000,    2.2361,    2.8284).finished();

  Point2 origin(0, 0);
  double cell_size = 1.0;

  sdf2 = PlanarSDF(origin, cell_size, field2);
}

/* ************************************************************************** */
TEST_UNSAFE(ObstaclePlanarSDFFactorGPArm, error) {

  // 2 link simple example
  Pose3 arm_base(Rot3(), Point3(0.5, 1.5, 0));
  Vector2 a(1, 2), alpha(0, 0), d(0, 0);
  Arm abs_arm(2, a, alpha, d, arm_base);

  // body info, three spheres
  BodySphereVector body_spheres;
  const double r = 0.5;
  body_spheres.push_back(BodySphere(0, r, Point3(-1, 0, 0)));
  body_spheres.push_back(BodySphere(0, r, Point3(0, 0, 0)));
  body_spheres.push_back(BodySphere(1, r, Point3(-1, 0, 0)));
  body_spheres.push_back(BodySphere(1, r, Point3(0, 0, 0)));
  ArmModel arm(abs_arm, body_spheres);

  // GP settings
  SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(2, 1.0);
  double delta_t = 0.1, tau = 0.025;

  double obs_eps = 1.0;
  ObstaclePlanarSDFFactorGPArm factor(0, 0, 0, 0, arm, sdf2, 1.0, obs_eps,
      Qc_model, delta_t, tau);

  // just check cost of two link joint
  Vector2 q1, q2, qdot1, qdot2;
  Vector err_act, err_exp, sdf_exp;
  Matrix H1_exp, H1_act, H2_exp, H2_act, H3_exp, H3_act, H4_exp, H4_act;


  // origin zero  and stationary case
  q1    = Vector2(0, 0);
  q2    = Vector2(0, 0);
  qdot1 = Vector2(0, 0);
  qdot2 = Vector2(0, 0);
  err_act = factor.evaluateError(q1, qdot1, q2, qdot2, H1_act, H2_act, H3_act, H4_act);
  sdf_exp = (Vector(4) << 1.662575, 0.60355, 0, 0).finished();
  err_exp = convertSDFtoErr(sdf_exp, obs_eps+r);
  H1_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, _1, qdot1, q2, qdot2)), q1, 1e-6);
  H2_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, q1, _1, q2, qdot2)), qdot1, 1e-6);
  H3_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, q1, qdot1, _1, qdot2)), q2, 1e-6);
  H4_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, q1, qdot1, q2, _1)), qdot2, 1e-6);
  EXPECT(assert_equal(err_exp, err_act, 1e-6));
  EXPECT(assert_equal(H1_exp, H1_act, 1e-6));
  EXPECT(assert_equal(H2_exp, H2_act, 1e-6));
  EXPECT(assert_equal(H3_exp, H3_act, 1e-6));
  EXPECT(assert_equal(H4_exp, H4_act, 1e-6));


  // 45 deg case
  q1    = Vector2(0, 0);
  q2    = Vector2(M_PI, 0);
  qdot1 = Vector2(M_PI * 10, 0);
  qdot2 = Vector2(M_PI * 10, 0);
  err_act = factor.evaluateError(q1, qdot1, q2, qdot2, H1_act, H2_act, H3_act, H4_act);
  sdf_exp = (Vector(4) << 1.662575, 0.585786437626905, -0.8284271247461900, -1.235281374238570).finished();
  err_exp = convertSDFtoErr(sdf_exp, obs_eps+r);
  H1_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, _1, qdot1, q2, qdot2)), q1, 1e-6);
  H2_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, q1, _1, q2, qdot2)), qdot1, 1e-6);
  H3_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, q1, qdot1, _1, qdot2)), q2, 1e-6);
  H4_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, q1, qdot1, q2, _1)), qdot2, 1e-6);
  EXPECT(assert_equal(err_exp, err_act, 1e-6));
  EXPECT(assert_equal(H1_exp, H1_act, 1e-6));
  EXPECT(assert_equal(H2_exp, H2_act, 1e-6));
  EXPECT(assert_equal(H3_exp, H3_act, 1e-6));
  EXPECT(assert_equal(H4_exp, H4_act, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}


