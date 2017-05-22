/**
 *  @file   testPointRobotModel.cpp
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

 #include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <gpmp2/kinematics/PointRobotModel.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

// fk wrapper
Pose3 fkpose(const PointRobot& pR, const Vector& jp, const Vector& jv, size_t i) {
  vector<Pose3> pos;
  vector<Vector3> vel;
  pR.forwardKinematics(jp, jv, pos, vel);
  return pos[i];
}

Vector3 fkvelocity(const PointRobot& pR, const Vector& jp, const Vector& jv, size_t i) {
  vector<Pose3> pos;
  vector<Vector3> vel;
  pR.forwardKinematics(jp, jv, pos, vel);
  return vel[i];
}

// sphere position wrapper
Point3 sph_pos_wrapper_batch(const PointRobotModel& pR_model, const Vector& jp, size_t i) {
  vector<Point3> pos;
  pR_model.sphereCenters(jp, pos);
  return pos[i];
}

Point3 sph_pos_wrapper_single(const PointRobotModel& pR_model, const Vector& jp, size_t i) {
  return pR_model.sphereCenter(i, jp);
}

/* ************************************************************************** */
TEST(PointRobot, 2DExample) {

  // 2D point robot
  PointRobot pR(2,1);
  Vector2 p, v;
  vector<Pose3> pvec_exp, pvec_act;
  vector<Vector3> vvec_exp, vvec_act;
  vector<Matrix> vJp_exp, vJp_act, vJv_exp, vJv_act;
  vector<Matrix> pJp_exp, pJp_act;

  // random position and velocity
  p = Vector2(1.0, 2.0);
  v = Vector2(0.1, 0.2);
  Vector vdymc = Vector(v);
  pR.forwardKinematics(p, vdymc, pvec_act, vvec_act, pJp_act, vJp_act, vJv_act);

  pvec_exp.clear();
  pvec_exp.push_back(Pose3(Rot3(), Point3(p(0), p(1), 0)));
  vvec_exp.clear();
  vvec_exp.push_back(Vector3(v(0), v(1), 0));

  pJp_exp.clear();
  pJp_exp.push_back(numericalDerivative11(boost::function<Pose3(const Vector2&)>(
      boost::bind(&fkpose, pR, _1, v, size_t(0))), p, 1e-6));
  vJp_exp.clear();
  vJp_exp.push_back(numericalDerivative11(boost::function<Vector3(const Vector2&)>(
      boost::bind(&fkvelocity, pR, _1, v, size_t(0))), p, 1e-6));
  vJv_exp.clear();
  vJv_exp.push_back(numericalDerivative11(boost::function<Vector3(const Vector2&)>(
      boost::bind(&fkvelocity, pR, p, _1, size_t(0))), v, 1e-6));

  EXPECT(assert_equal(pvec_exp[0], pvec_act[0], 1e-9));
  EXPECT(assert_equal(vvec_exp[0], vvec_act[0], 1e-9));
  EXPECT(assert_equal(pJp_exp[0], pJp_act[0], 1e-6));
  EXPECT(assert_equal(vJp_exp[0], vJp_act[0], 1e-6));
  EXPECT(assert_equal(vJv_exp[0], vJv_act[0], 1e-6));
}

/* ************************************************************************** */
TEST(PointRobotModel, 2DExample) {

  // 2D point robot
  PointRobot pR(2,1);
  Vector2 p;

  vector<Point3> sph_centers_exp, sph_centers_act;
  vector<Matrix> J_center_q_act;
  Matrix Jcq_exp, Jcq_act;

  // body spheres
  BodySphereVector body_spheres;
  body_spheres.push_back(BodySphere(0, 1.5, Point3(0, 0, 0)));
  const size_t nr_sph = body_spheres.size();

  PointRobotModel pR_model(pR, body_spheres);

  // random position
  p = Vector2(1, 2);
  sph_centers_exp.clear();
  sph_centers_exp.push_back(Point3(1, 2, 0));

  pR_model.sphereCenters(p, sph_centers_act, J_center_q_act);

  for (size_t i = 0; i < nr_sph; i++) {
    EXPECT(assert_equal(sph_centers_exp[i], sph_centers_act[i]));
    Jcq_exp = numericalDerivative11(boost::function<Point3(const Vector2&)>(
          boost::bind(&sph_pos_wrapper_batch, pR_model, _1, i)), p, 1e-6);
    EXPECT(assert_equal(Jcq_exp, J_center_q_act[i], 1e-9));
    EXPECT(assert_equal(sph_centers_exp[i], pR_model.sphereCenter(i, p, Jcq_act)));
    Jcq_exp = numericalDerivative11(boost::function<Point3(const Vector2&)>(
          boost::bind(&sph_pos_wrapper_single, pR_model, _1, i)), p, 1e-6);
    EXPECT(assert_equal(Jcq_exp, Jcq_act, 1e-9));
  }
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
