/**
*  @file testVehicleDynamics.cpp
*  @author Jing Dong
**/

#include <CppUnitLite/TestHarness.h>

#include <gpmp2/dynamics/VehicleDynamics.h>

#include <gtsam/base/numericalDerivative.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
TEST(VehicleDynamics, simple2DVehicleDynamics) {

  Pose2 p;
  Vector3 v;
  double cexp, cact;
  Matrix13 Hpexp, Hvexp, Hpact, Hvact;


  // zero speed zero heading
  p = Pose2();
  v = Vector3::Zero();
  cexp = 0;
  Hpexp = numericalDerivative11(boost::function<double(const Pose2&)>(
        boost::bind(simple2DVechileDyanmics, _1, v, boost::none, boost::none)), p);
  Hvexp = numericalDerivative11(boost::function<double(const Vector3&)>(
        boost::bind(simple2DVechileDyanmics, p, _1, boost::none, boost::none)), v);
  cact = simple2DVechileDyanmics(p, v, Hpact, Hvact);
  EXPECT_DOUBLES_EQUAL(cexp, cact, 1e-9);
  EXPECT(assert_equal(Hpexp, Hpact, 1e-6));
  EXPECT(assert_equal(Hvexp, Hvact, 1e-6));

  // zero speed non-zero heading
  p = Pose2(1.0, 2.0, 0.3);
  v = (Vector3() << 0, 0, 1.2).finished();
  cexp = 0;
  Hpexp = numericalDerivative11(boost::function<double(const Pose2&)>(
        boost::bind(simple2DVechileDyanmics, _1, v, boost::none, boost::none)), p);
  Hvexp = numericalDerivative11(boost::function<double(const Vector3&)>(
        boost::bind(simple2DVechileDyanmics, p, _1, boost::none, boost::none)), v);
  cact = simple2DVechileDyanmics(p, v, Hpact, Hvact);
  EXPECT_DOUBLES_EQUAL(cexp, cact, 1e-9);
  EXPECT(assert_equal(Hpexp, Hpact, 1e-6));
  EXPECT(assert_equal(Hvexp, Hvact, 1e-6));

  // non-zero speed zero heading
  p = Pose2(1.0, 2.0, M_PI * 0.75);
  v = (Vector3() << -1, 1, 1.2).finished();
  cexp = 0;
  Hpexp = numericalDerivative11(boost::function<double(const Pose2&)>(
        boost::bind(simple2DVechileDyanmics, _1, v, boost::none, boost::none)), p);
  Hvexp = numericalDerivative11(boost::function<double(const Vector3&)>(
        boost::bind(simple2DVechileDyanmics, p, _1, boost::none, boost::none)), v);
  cact = simple2DVechileDyanmics(p, v, Hpact, Hvact);
  EXPECT_DOUBLES_EQUAL(cexp, cact, 1e-9);
  EXPECT(assert_equal(Hpexp, Hpact, 1e-6));
  EXPECT(assert_equal(Hvexp, Hvact, 1e-6));

  // non-zero speed non-zero heading
  p = Pose2(1.0, 2.0, M_PI * 0.75);
  v = (Vector3() << 0, 1, 1.2).finished();
  cexp = -0.707106781186548;
  Hpexp = numericalDerivative11(boost::function<double(const Pose2&)>(
        boost::bind(simple2DVechileDyanmics, _1, v, boost::none, boost::none)), p);
  Hvexp = numericalDerivative11(boost::function<double(const Vector3&)>(
        boost::bind(simple2DVechileDyanmics, p, _1, boost::none, boost::none)), v);
  cact = simple2DVechileDyanmics(p, v, Hpact, Hvact);
  EXPECT_DOUBLES_EQUAL(cexp, cact, 1e-9);
  EXPECT(assert_equal(Hpexp, Hpact, 1e-6));
  EXPECT(assert_equal(Hvexp, Hvact, 1e-6));

  // random stuff for jacobians
  p = Pose2(34.6, -31.2, 1.75);
  v = (Vector3() << 14.3, 6.7, 22.2).finished();
  Hpexp = numericalDerivative11(boost::function<double(const Pose2&)>(
        boost::bind(simple2DVechileDyanmics, _1, v, boost::none, boost::none)), p);
  Hvexp = numericalDerivative11(boost::function<double(const Vector3&)>(
        boost::bind(simple2DVechileDyanmics, p, _1, boost::none, boost::none)), v);
  cact = simple2DVechileDyanmics(p, v, Hpact, Hvact);
  //EXPECT_DOUBLES_EQUAL(cexp, cact, 1e-9);
  EXPECT(assert_equal(Hpexp, Hpact, 1e-6));
  EXPECT(assert_equal(Hvexp, Hvact, 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}