/**
 *  @file  Pose2MobileArm.cpp
 *  @brief Abstract plannar mobile manipulator, Pose2 + Arm
 *  @author Jing Dong
 *  @date  Oct 4, 2016
 **/

#include <gpmp2/kinematics/Pose2MobileArm.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>

#include <iostream>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
Pose2MobileArm::Pose2MobileArm(const Arm& arm, const gtsam::Pose3& base_T_arm) :
    Base(arm.dof() + 3, arm.dof() + 1), base_T_arm_(base_T_arm), arm_(arm) {

  // check arm base pose, warn if non-zero
  if (!arm_.base_pose().equals(Pose3::identity(), 1e-6))
    cout << "[Pose2MobileArm] WARNING: Arm has non-zero base pose, this base pose will be override. "
        "Set the base_T_arm in Pose2MobileArm." << endl;
}

/* ************************************************************************** */
void Pose2MobileArm::forwardKinematics(
    const Pose2Vector& p, boost::optional<const gtsam::Vector&> v,
    std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector3>&> vx,
    boost::optional<std::vector<gtsam::Matrix>&> J_px_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_v) const {

  if (v)
    throw runtime_error("[Pose2MobileArm] TODO: velocity not implemented");

  if (!v && (vx || J_vx_p || J_vx_v))
    throw runtime_error("[Pose2MobileArm] ERROR: only ask for velocity in workspace given velocity in "
        "configuration space");

  // space for output
  px.resize(nr_links());
  if (vx) vx->resize(nr_links());
  if (J_px_p) J_px_p->assign(nr_links(), Matrix::Zero(6, dof()));
  if (J_vx_p) J_vx_p->assign(nr_links(), Matrix::Zero(3, dof()));
  if (J_vx_v) J_vx_v->assign(nr_links(), Matrix::Zero(3, dof()));

  // vehicle & arm base pose
  Pose3 veh_base, arm_base;
  Matrix63 Hveh_base, Harm_base;
  if (J_px_p || J_vx_p || J_vx_v) {
    veh_base = computeBasePose3(p.pose(), Hveh_base);
    arm_base = computeBaseTransPose3(p.pose(), base_T_arm_, Harm_base);
  } else {
    veh_base = computeBasePose3(p.pose());
    arm_base = computeBaseTransPose3(p.pose(), base_T_arm_);
  }


  // call arm pose and velocity, for arm links
  // px[0] = base_pose3; px[i] = arm_base * px_arm[i-1]
  // vx[0] = v(0:1,0); vx[i] = vx[0] + angular x arm_base_pos + arm_base_rot * vx_arm[i-1]

  // veh base link
  px[0] = veh_base;
  if (J_px_p) (*J_px_p)[0].block<6,3>(0,0) = Hveh_base;
  if (vx) {
    (*vx)[0] = Vector3((*v)[0], (*v)[1], 0.0);
    // (*J_vx_p)[0] is zero
    if (J_vx_v)
      (*J_vx_v)[0].block<2,2>(0,0) = Matrix2::Identity();
  }

  // arm links
  vector<Pose3> armjpx;
  vector<Vector3> armjvx;
  vector<Matrix> Jarm_jpx_jp, Jarm_jvx_jp, Jarm_jvx_jv;

  arm_.updateBasePose(arm_base);
  if (v) {
    const Vector varm = v->tail(arm_.dof());
    arm_.forwardKinematics(p.configuration(), boost::optional<const Vector&>(varm),
        armjpx, vx ? boost::optional<vector<Vector3>&>(armjvx) : boost::none,
        J_px_p ? boost::optional<vector<Matrix>&>(Jarm_jpx_jp) : boost::none,
        J_vx_p ? boost::optional<vector<Matrix>&>(Jarm_jvx_jp) : boost::none,
        J_vx_v ? boost::optional<vector<Matrix>&>(Jarm_jvx_jv) : boost::none);
  } else {
    arm_.forwardKinematics(p.configuration(), boost::none,
        armjpx, vx ? boost::optional<vector<Vector3>&>(armjvx) : boost::none,
        J_px_p ? boost::optional<vector<Matrix>&>(Jarm_jpx_jp) : boost::none);
  }

  for (size_t i = 0; i < arm_.dof(); i++) {
    px[i+1] = armjpx[i];
    if (J_px_p) {
      // see compose's jacobian
      (*J_px_p)[i+1].block<6,3>(0,0) = (armjpx[i].inverse() * arm_base).AdjointMap() * Harm_base;
      (*J_px_p)[i+1].block(0,3,6,arm_.dof()) = Jarm_jpx_jp[i];
    }
    if (vx) {
      //(*vx)[i+1] = Vector3((*v)[0], (*v)[1], 0.0) +  armjvx[i];
    }
  }

}

} // namespace gpmp2

