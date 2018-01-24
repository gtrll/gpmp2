/**
 *  @file   testPose2MobileBase.cpp
 *  @author Mustafa Mukadam
 *  @date   Jan 22, 2018
 **/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/kinematics/Pose2MobileBase.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
// fk wrapper
Pose3 fkpose(const Pose2MobileBase& r, const Pose2& p, const Vector& v) {
  vector<Pose3> pos;
  r.forwardKinematics(p, boost::none, pos, boost::none);
  return pos[0];
}

Vector3 fkvelocity(const Pose2MobileBase& r, const Pose2& p, const Vector& v) {
  vector<Pose3> pos;
  vector<Vector3> vel;
  r.forwardKinematics(p, v, pos, vel);
  return vel[0];
}

TEST(Pose2MobileBase, Example) {

  Pose2MobileBase robot;

  Pose2 q;
  Vector3 qdot;
  Vector qdymc;
  vector<Pose3> pvec_exp, pvec_act;
  vector<Vector3> vvec_exp, vvec_act;
  vector<Matrix> vJp_exp, vJp_act, vJv_exp, vJv_act;
  vector<Matrix> pJp_exp, pJp_act;

  // origin with zero vel
  q = Pose2();
  qdot = (Vector3() << 0, 0, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3());
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(0, 0, 0));
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2&)>(
      boost::bind(&fkpose, robot, _1, qdot)), q, 1e-6));
  // vJp_exp.clear();
  // vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2&)>(
  //     boost::bind(&fkvelocity, robot, _1, qdot)), q, 1e-6));
  // vJv_exp.clear();
  // vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector3&)>(
  //     boost::bind(&fkvelocity, robot, q, _1)), qdot, 1e-6));
  //robot.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  robot.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);
  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  // EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));

  // origin with none zero vel
  q = Pose2();
  qdot = (Vector3() << 1, 0, 1).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3());
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(1, 0, 1));
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2&)>(
      boost::bind(&fkpose, robot, _1, qdot)), q, 1e-6));
  // vJp_exp.clear();
  // vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2&)>(
  //     boost::bind(&fkvelocity, robot, _1, qdot)), q, 1e-6));
  // vJv_exp.clear();
  // vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector3&)>(
  //     boost::bind(&fkvelocity, robot, q, _1)), qdot, 1e-6));
  //robot.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  robot.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);
  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  // EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));

  // non-zero pose
  q = Pose2(2.0, -1.0, M_PI/2.0);
  qdot = (Vector3() << 0, 0, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/2.0, 0, 0), Point3(2.0, -1.0, 0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(0, 0, 0));
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2&)>(
      boost::bind(&fkpose, robot, _1, qdot)), q, 1e-6));
  // vJp_exp.clear();
  // vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2&)>(
  //     boost::bind(&fkvelocity, robot, _1, qdot)), q, 1e-6));
  // vJv_exp.clear();
  // vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector3&)>(
  //     boost::bind(&fkvelocity, robot, q, _1)), qdot, 1e-6));
  //robot.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  robot.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);
  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  // EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));

  // non-zero pose and vel
  q = Pose2(2.0, -1.0, M_PI/2.0);
  qdot = (Vector3() << 1, 0, 1).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/2.0, 0, 0), Point3(2.0, -1.0, 0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(1, 0, 1));
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2&)>(
      boost::bind(&fkpose, robot, _1, qdot)), q, 1e-6));
  // vJp_exp.clear();
  // vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2&)>(
  //     boost::bind(&fkvelocity, robot, _1, qdot)), q, 1e-6));
  // vJv_exp.clear();
  // vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector3&)>(
  //     boost::bind(&fkvelocity, robot, q, _1)), qdot, 1e-6));
  //robot.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  robot.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);
  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  // EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));

  // other values to check Jacobians
  q = Pose2(2.3, -1.2, 0.1);
  qdot = (Vector3() << 3.2, 4.9, -3.3).finished();
  qdymc = qdot;
  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2&)>(
      boost::bind(&fkpose, robot, _1, qdot)), q, 1e-6));
  // vJp_exp.clear();
  // vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2&)>(
  //     boost::bind(&fkvelocity, robot, _1, qdot)), q, 1e-6));
  // vJv_exp.clear();
  // vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector3&)>(
  //     boost::bind(&fkvelocity, robot, q, _1)), qdot, 1e-6));
  //robot.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  robot.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  // EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
