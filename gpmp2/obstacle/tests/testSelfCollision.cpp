/**
 * @file   testSelfCollisionArm.cpp
 * @author Mustafa Mukadam
 **/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/obstacle/SelfCollisionArm.h>

#include <iostream>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(SelfCollisionArm, error) {

  // 3 link arm
  Vector3 a(1, 1, 1), alpha(0, 0, 0), d(0, 0, 0);
  Arm abs_arm(3, a, alpha, d);
  BodySphereVector body_spheres;
  body_spheres.push_back(BodySphere(0, 0.1, Point3(-0.5, 0, 0)));
  body_spheres.push_back(BodySphere(1, 0.1, Point3(-0.5, 0, 0)));
  body_spheres.push_back(BodySphere(1, 0.1, Point3(0, 0, 0)));
  body_spheres.push_back(BodySphere(2, 0.1, Point3(0, 0, 0)));
  ArmModel arm(abs_arm, body_spheres);

  Matrix data(2, 4);
  // sphere A id, sphere B id, epsilon, sigma
  data << 
    0, 1, 2.0, 0.1,
    2, 3, 5.0, 0.1;
  SelfCollisionArm factor = SelfCollisionArm(0, arm, data);

  Vector actual, expect;
  Matrix H_exp, H_act;

  Vector3 q;
  q = Vector3(0.0, M_PI/2.0, 0.0);
  actual = factor.evaluateError(q, H_act);
  expect = (Vector(2) << 1.4928932188134527, 4.2).finished();
  H_exp = numericalDerivative11(boost::function<Vector2(const Vector3&)>(
    boost::bind(&SelfCollisionArm::evaluateError, factor, _1, boost::none)), q, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_exp, H_act, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
