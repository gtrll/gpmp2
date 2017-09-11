/**
 *  @file  Pose2Mobile2Arms.cpp
 *  @brief Abstract plannar mobile manipulator, Pose2 + Arm
 *  @author Jing Dong
 *  @date  Aug 20, 2016
 **/

#include <gpmp2/kinematics/Pose2Mobile2Arms.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>

#include <iostream>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
Pose2Mobile2Arms::Pose2Mobile2Arms(const Arm& arm1, const Arm& arm2, 
    const gtsam::Pose3& base_T_arm1, const gtsam::Pose3& base_T_arm2) :
    Base(arm1.dof() + arm2.dof() + 3, arm1.dof() + arm2.dof() + 1), 
    base_T_arm1_(base_T_arm1), base_T_arm2_(base_T_arm2), arm1_(arm1), arm2_(arm2) {

  // check arm base pose, warn if non-zero
  if (!arm1_.base_pose().equals(Pose3::identity(), 1e-6) || 
      !arm2_.base_pose().equals(Pose3::identity(), 1e-6))
    cout << "[Pose2Mobile2Arms] WARNING: Arm has non-zero base pose, this base pose will be override. "
        "Set the base_T_arm in Pose2MobileArm." << endl;
}

/* ************************************************************************** */
void Pose2Mobile2Arms::forwardKinematics(
    const Pose2Vector& p, boost::optional<const gtsam::Vector&> v,
    std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector3>&> vx,
    boost::optional<std::vector<gtsam::Matrix>&> J_px_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_v) const {


  if (v)
    throw runtime_error("[Pose2Mobile2Arms] TODO: velocity not implemented");

  if (!v && (vx || J_vx_p || J_vx_v))
    throw runtime_error("[Pose2Mobile2Arms] ERROR: only ask for velocity in workspace given velocity in "
        "configuration space");

  // space for output
  px.resize(nr_links());
  if (vx) vx->resize(nr_links());
  if (J_px_p) J_px_p->assign(nr_links(), Matrix::Zero(6, dof()));
  if (J_vx_p) J_vx_p->assign(nr_links(), Matrix::Zero(3, dof()));
  if (J_vx_v) J_vx_v->assign(nr_links(), Matrix::Zero(3, dof()));

  // vehicle & arm base pose
  Pose3 veh_base, arm1_base, arm2_base;
  Matrix63 Hveh_base, Harm1_base, Harm2_base;
  if (J_px_p || J_vx_p || J_vx_v) {
    veh_base = computeBasePose3(p.pose(), Hveh_base);
    arm1_base = computeBaseTransPose3(p.pose(), base_T_arm1_, Harm1_base);
    arm2_base = computeBaseTransPose3(p.pose(), base_T_arm2_, Harm2_base);
  } else {
    veh_base = computeBasePose3(p.pose());
    arm1_base = computeBaseTransPose3(p.pose(), base_T_arm1_);
    arm2_base = computeBaseTransPose3(p.pose(), base_T_arm2_);
  }

  // call arm pose and velocity, for arm links
  // px[0] = base_pose3; px[i] = arm_base * px_arm[i-1]
  // vx[0] = v(0:1,0); vx[i] = vx[0] + angular x arm_base_pos + arm_base_rot * vx_arm[i-1]

  // veh base link
  px[0] = veh_base;
  if (J_px_p) (*J_px_p)[0].block<6,3>(0,0) = Hveh_base;

  // arm1 and arm2 links
  vector<Pose3> arm1jpx, arm2jpx;
  vector<Matrix> Jarm1_jpx_jp, Jarm2_jpx_jp;

  arm1_.updateBasePose(arm1_base);
  arm1_.forwardKinematics(p.configuration().head(arm1_.dof()), boost::none, arm1jpx, boost::none,
      J_px_p ? boost::optional<vector<Matrix>&>(Jarm1_jpx_jp) : boost::none);
  arm2_.updateBasePose(arm2_base);
  arm2_.forwardKinematics(p.configuration().tail(arm2_.dof()), boost::none, arm2jpx, boost::none,
      J_px_p ? boost::optional<vector<Matrix>&>(Jarm2_jpx_jp) : boost::none);

  for (size_t i = 0; i < arm1_.dof(); i++) {
    px[i+1] = arm1jpx[i];
    if (J_px_p) {
      // see compose's jacobian
      (*J_px_p)[i+1].block<6,3>(0,0) = (arm1jpx[i].inverse() * arm1_base).AdjointMap() * Harm1_base;
      (*J_px_p)[i+1].block(0,3,6,arm1_.dof()) = Jarm1_jpx_jp[i];
    }
  }
  for (size_t i = 0; i < arm2_.dof(); i++) {
    px[i+1+arm1_.dof()] = arm2jpx[i];
    if (J_px_p) {
      // see compose's jacobian
      (*J_px_p)[i+1+arm1_.dof()].block<6,3>(0,0) = (arm2jpx[i].inverse() * arm2_base).AdjointMap() * Harm2_base;
      (*J_px_p)[i+1+arm1_.dof()].block(0,3+arm1_.dof(),6,arm2_.dof()) = Jarm2_jpx_jp[i];
    }
  }
}

} // namespace gpmp2

