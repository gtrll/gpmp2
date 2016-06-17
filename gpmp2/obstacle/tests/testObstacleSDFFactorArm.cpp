/**
*  @file testObstacleSDFFactorArm.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


typedef ObstacleSDFFactor<ArmModel> ObstacleSDFFactorArm;


inline Vector errorWrapper(const ObstacleSDFFactorArm& factor,
    const Vector& conf) {
  return factor.evaluateError(conf);
}

// convert sdf vector to hinge loss err vector
inline Vector convertSDFtoErr(const Vector& sdf, double eps) {
  Vector err_ori = 0.0 - sdf.array() + eps;
  return (err_ori.array() > 0.0).select(err_ori, Vector::Zero(err_ori.rows()));  // (R < s ? P : Q)
}

// data
SignedDistanceField sdf;

/* ************************************************************************** */
TEST(ObstacleSDFFactorArm, data) {

  double cell_size = 0.1;
  // zero orgin
  Point3 origin(0,0,0);
  vector<Matrix> field(3);

  field[0] = (Matrix(7,7) <<
      0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828,
      0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236,
      0.2000, 0.1000, -0.1000, -0.1000, -0.1000, 0.1000, 0.2000,
      0.2000, 0.1000, -0.1000, -0.1000, -0.1000, 0.1000, 0.2000,
      0.2000, 0.1000, -0.1000, -0.1000, -0.1000, 0.1000, 0.2000,
      0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236,
      0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828).finished();
  field[1] = (Matrix(7,7) <<
      0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000,
      0.2449, 0.1732, 0.1414, 0.1414, 0.1414, 0.1732, 0.2449,
      0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236,
      0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236,
      0.2236, 0.1414, 0.1000, 0.1000, 0.1000, 0.1414, 0.2236,
      0.2449, 0.1732, 0.1414, 0.1414, 0.1414, 0.1732, 0.2449,
      0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000).finished();
  field[2] = (Matrix(7,7) <<
      0.3464, 0.3000, 0.2828, 0.2828, 0.2828, 0.3000, 0.3464,
      0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000,
      0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828,
      0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828,
      0.2828, 0.2236, 0.2000, 0.2000, 0.2000, 0.2236, 0.2828,
      0.3000, 0.2449, 0.2236, 0.2236, 0.2236, 0.2449, 0.3000,
      0.3464, 0.3000, 0.2828, 0.2828, 0.2828, 0.3000, 0.3464).finished();

  sdf = SignedDistanceField(origin, cell_size, field);
}

/* ************************************************************************** */
TEST(ObstacleSDFFactorArm, error) {

  // 2 link simple example
  Pose3 arm_base(Rot3(), Point3(0.05, 0.15, 0.05));
  Vector2 a(0.1, 0.2), alpha(0, 0), d(0, 0);
  Arm abs_arm(2, a, alpha, d, arm_base);

  // body info, three spheres
  BodySphereVector body_spheres;
  const double r = 0.05;
  body_spheres.push_back(BodySphere(0, r, Point3(-0.1, 0, 0)));
  body_spheres.push_back(BodySphere(0, r, Point3(0, 0, 0)));
  body_spheres.push_back(BodySphere(1, r, Point3(-0.1, 0, 0)));
  body_spheres.push_back(BodySphere(1, r, Point3(0, 0, 0)));
  ArmModel arm(abs_arm, body_spheres);

  double obs_eps = 0.2;
  ObstacleSDFFactorArm factor(0, arm, sdf, 1.0, obs_eps);

  // just check cost of two link joint
  Vector2 q;
  Vector err_act, err_exp, sdf_exp;
  Matrix H1_exp, H1_act;


  // origin zero case
  q = Vector2(0, 0);
  err_act = factor.evaluateError(q, H1_act);
  sdf_exp = (Vector(4) << 0.1810125, 0.099675, 0.06035, 0.06035).finished();
  err_exp = convertSDFtoErr(sdf_exp, obs_eps+r);
  H1_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, _1)), q, 1e-6);
  EXPECT(assert_equal(err_exp, err_act, 1e-6));
  EXPECT(assert_equal(H1_exp, H1_act, 1e-6));

  // 45 deg case
  q = Vector2(M_PI/4.0, 0);
  err_act = factor.evaluateError(q, H1_act);
  sdf_exp = (Vector(4) << 0.1810125, 0.095702211510784, 0.01035442302156, 0).finished();
  err_exp = convertSDFtoErr(sdf_exp, obs_eps+r);
  H1_exp = numericalDerivative11(boost::function<Vector(const Vector2&)>(
      boost::bind(&errorWrapper, factor, _1)), q, 1e-6);
  EXPECT(assert_equal(err_exp, err_act, 1e-6));
  EXPECT(assert_equal(H1_exp, H1_act, 1e-6));
}


/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}


