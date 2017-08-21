/**
*  @file testPose2MobileArm.cpp
*  @author Jing Dong
*  @date Oct 5, 2016
**/

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/kinematics/Pose2MobileArm.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>
#include <gpmp2/geometry/numericalDerivativeDynamic.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;


/* ************************************************************************** */
// fk wrapper
Pose3 fkpose(const Pose2MobileArm& r, const Pose2Vector& p, const Vector& v, size_t i) {
  vector<Pose3> pos;
  vector<Vector3> vel;
  r.forwardKinematics(p, boost::none, pos, boost::none);
  return pos[i];
}

Vector3 fkvelocity(const Pose2MobileArm& r, const Pose2Vector& p, const Vector& v, size_t i) {
  vector<Pose3> pos;
  vector<Vector3> vel;
  r.forwardKinematics(p, v, pos, vel);
  return vel[i];
}

TEST(Pose2MobileArm, 2linkPlanarExamples) {

  // 2 link simple example, with none zero base poses
  Vector2 a(1, 1), alpha(0, 0), d(0, 0);
  Arm arm(2, a, alpha, d);
  Pose3 base_pose(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(1.0, 0.0, 2.0));
  Pose2MobileArm marm(arm, base_pose);

  Pose2Vector q;
  Vector5 qdot;
  Vector qdymc;
  vector<Pose3> pvec_exp, pvec_act;
  vector<Vector3> vvec_exp, vvec_act;
  vector<Matrix> vJp_exp, vJp_act, vJv_exp, vJv_act;
  vector<Matrix> pJp_exp, pJp_act;


  // origin with zero vel
  q = Pose2Vector(Pose2(), Vector2(0.0, 0.0));
  qdot = (Vector5() << 0, 0, 0, 0, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3());
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(1.707106781186548, 0.707106781186548, 2.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(2.414213562373095, 1.414213562373095, 2.0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(0))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(1))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(2))), q, 1e-6));
/*
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(0))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(1))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(2))), q, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(0))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(1))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(2))), qdot, 1e-6));
*/
  //marm.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  marm.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);

  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
/*
  EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(vvec_exp[1], vvec_act[1], 1e-9));
  EXPECT(assert_equal(vvec_exp[2], vvec_act[2], 1e-9));
*/
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
/*
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[1], vJp_act[1], 1e-6));
  EXPECT(assert_equal(vJp_exp[2], vJp_act[2], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[1], vJv_act[1], 1e-6));
  EXPECT(assert_equal(vJv_exp[2], vJv_act[2], 1e-6));
*/

  // origin with none zero vel of vehl
  q = Pose2Vector(Pose2(), Vector2(0.0, 0.0));
  qdot = (Vector5() << 1, 0, 1, 0, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3());
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(1.707106781186548, 0.707106781186548, 2.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(2.414213562373095, 1.414213562373095, 2.0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(1, 0, 1));
  vvec_exp.push_back(Vector3(1, 0, 1));
  vvec_exp.push_back(Vector3(1, 0, 1));

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(0))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(1))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(2))), q, 1e-6));
/*
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(0))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(1))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(2))), q, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(0))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(1))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(2))), qdot, 1e-6));
*/
  //marm.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  marm.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);

  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
/*
  EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(vvec_exp[1], vvec_act[1], 1e-9));
  EXPECT(assert_equal(vvec_exp[2], vvec_act[2], 1e-9));
*/
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
/*
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[1], vJp_act[1], 1e-6));
  EXPECT(assert_equal(vJp_exp[2], vJp_act[2], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[1], vJv_act[1], 1e-6));
  EXPECT(assert_equal(vJv_exp[2], vJv_act[2], 1e-6));
*/

  // origin with none zero vel of arm
  q = Pose2Vector(Pose2(), Vector2(0.0, 0.0));
  qdot = (Vector5() << 0, 0, 0, 1, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3());
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(1.707106781186548, 0.707106781186548, 2.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/4.0, 0, 0), Point3(2.414213562373095, 1.414213562373095, 2.0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(0))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(1))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(2))), q, 1e-6));
/*
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(0))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(1))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(2))), q, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(0))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(1))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(2))), qdot, 1e-6));
*/
  //marm.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  marm.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);

  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
/*
  EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(vvec_exp[1], vvec_act[1], 1e-9));
  EXPECT(assert_equal(vvec_exp[2], vvec_act[2], 1e-9));
*/
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
/*
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[1], vJp_act[1], 1e-6));
  EXPECT(assert_equal(vJp_exp[2], vJp_act[2], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[1], vJv_act[1], 1e-6));
  EXPECT(assert_equal(vJv_exp[2], vJv_act[2], 1e-6));
*/


  // origin with none zero pose
  q = Pose2Vector(Pose2(2.0, -1.0, M_PI/2.0), Vector2(M_PI/4.0, 0.0));
  qdot = (Vector5() << 0, 0, 0, 0, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/2.0, 0, 0), Point3(2.0, -1.0, 0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI, 0, 0), Point3(1.0, 0.0, 2.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI, 0, 0), Point3(0.0, 0.0, 2.0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(0))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(1))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(2))), q, 1e-6));
/*
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(0))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(1))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(2))), q, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(0))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(1))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(2))), qdot, 1e-6));
*/
  //marm.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  marm.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);

  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
/*
  EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(vvec_exp[1], vvec_act[1], 1e-9));
  EXPECT(assert_equal(vvec_exp[2], vvec_act[2], 1e-9));
*/
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
/*
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[1], vJp_act[1], 1e-6));
  EXPECT(assert_equal(vJp_exp[2], vJp_act[2], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[1], vJv_act[1], 1e-6));
  EXPECT(assert_equal(vJv_exp[2], vJv_act[2], 1e-6));
*/

  // origin with none zero pose and velocity
  q = Pose2Vector(Pose2(2.0, -1.0, M_PI/2.0), Vector2(M_PI/4.0, 0.0));
  qdot = (Vector5() << 0, 0, 0, 0, 0).finished();
  qdymc = qdot;
  pvec_exp.clear();
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI/2.0, 0, 0), Point3(2.0, -1.0, 0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI, 0, 0), Point3(1.0, 0.0, 2.0)));
  pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI, 0, 0), Point3(0.0, 0.0, 2.0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));
  vvec_exp.push_back(Vector3(0, 0, 0));

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(0))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(1))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(2))), q, 1e-6));
/*
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(0))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(1))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(2))), q, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(0))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(1))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(2))), qdot, 1e-6));
*/
  //marm.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  marm.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);

  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(pvec_exp[1], pvec_act[1], 1e-9));
  EXPECT(assert_equal(pvec_exp[2], pvec_act[2], 1e-9));
/*
  EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(vvec_exp[1], vvec_act[1], 1e-9));
  EXPECT(assert_equal(vvec_exp[2], vvec_act[2], 1e-9));
*/
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
/*
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[1], vJp_act[1], 1e-6));
  EXPECT(assert_equal(vJp_exp[2], vJp_act[2], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[1], vJv_act[1], 1e-6));
  EXPECT(assert_equal(vJv_exp[2], vJv_act[2], 1e-6));
*/

  // random stuff for jacobians
  q = Pose2Vector(Pose2(2.3, -1.2, 0.1), Vector2(1.2, -0.9));
  qdot = (Vector5() << 3.2, 4.9, -3.3, -2.1, 0.1).finished();
  qdymc = qdot;

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(0))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(1))), q, 1e-6));
  pJp_exp.push_back(numericalDerivativeDynamic(boost::function<Pose3(const Pose2Vector&)>(
      boost::bind(&fkpose, marm, _1, qdot, size_t(2))), q, 1e-6));
/*
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(0))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(1))), q, 1e-6));
  vJp_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Pose2Vector&)>(
      boost::bind(&fkvelocity, marm, _1, qdot, size_t(2))), q, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(0))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(1))), qdot, 1e-6));
  vJv_exp.push_back(numericalDerivativeDynamic(boost::function<Vector3(const Vector5&)>(
      boost::bind(&fkvelocity, marm, q, _1, size_t(2))), qdot, 1e-6));
*/
  //marm.forwardKinematics(q, qdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);
  marm.forwardKinematics(q, boost::none, pvec_act, boost::none, pJp_act);

  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(pJp_exp[1], pJp_act[1], 1e-6));
  EXPECT(assert_equal(pJp_exp[2], pJp_act[2], 1e-6));
/*
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[1], vJp_act[1], 1e-6));
  EXPECT(assert_equal(vJp_exp[2], vJp_act[2], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[1], vJv_act[1], 1e-6));
  EXPECT(assert_equal(vJv_exp[2], vJv_act[2], 1e-6));
*/

}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
