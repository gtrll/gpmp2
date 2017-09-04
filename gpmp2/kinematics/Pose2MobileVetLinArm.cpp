/**
 *  @file  Pose2MobileVetLinArm.cpp
 *  @brief Abstract plannar mobile manipulator, Arm on a vetical linear actuator
 *  @author Jing Dong
 *  @date  Aug 18, 2017
 **/

#include <gpmp2/kinematics/Pose2MobileVetLinArm.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>

#include <iostream>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
Pose2MobileVetLinArm::Pose2MobileVetLinArm(const Arm& arm, const gtsam::Pose3& base_T_torso, 
    const gtsam::Pose3& torso_T_arm, bool reverse_linact) : 
    Base(arm.dof() + 4, arm.dof() + 2), base_T_torso_(base_T_torso), torso_T_arm_(torso_T_arm), 
    reverse_linact_(reverse_linact), arm_(arm) {

  // check arm base pose, warn if non-zero
  if (!arm_.base_pose().equals(Pose3::identity(), 1e-6))
  cout << "[Pose2MobileArm] WARNING: Arm has non-zero base pose, this base pose will be override. "
      "Set the base_T_torso and torso_T_arm in Pose2MobileArm." << endl;
}

/* ************************************************************************** */
void Pose2MobileVetLinArm::forwardKinematics(const Pose2Vector& p, 
    boost::optional<const gtsam::Vector&> v,
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
  Pose3 veh_base, tso_base, arm_base;
  Matrix63 Hveh_base;
  Matrix64 Htso_base, Harm_base;
  if (J_px_p || J_vx_p || J_vx_v) {
    veh_base = computeBasePose3(p.pose(), Hveh_base);
    tso_base = liftBasePose3(p.pose(), p.configuration()(0), base_T_torso_, reverse_linact_, 
        Htso_base);
    Matrix6 H_tso_comp;
    arm_base = tso_base.compose(torso_T_arm_, H_tso_comp);
    Harm_base = H_tso_comp * Htso_base;

  } else {
    veh_base = computeBasePose3(p.pose());
    tso_base = liftBasePose3(p.pose(), p.configuration()(0), base_T_torso_, reverse_linact_);
    arm_base = tso_base.compose(torso_T_arm_);
  }

  // veh base link
  px[0] = veh_base;
  if (J_px_p) (*J_px_p)[0].block<6,3>(0,0) = Hveh_base;

  // torso link
  px[1] = tso_base;
  if (J_px_p) (*J_px_p)[1].block<6,4>(0,0) = Htso_base;

  // arm links
  vector<Pose3> armjpx;
  vector<Matrix> Jarm_jpx_jp;

  arm_.updateBasePose(arm_base);
  arm_.forwardKinematics(p.configuration().tail(arm_.dof()), boost::none, armjpx, boost::none,
      J_px_p ? boost::optional<vector<Matrix>&>(Jarm_jpx_jp) : boost::none);

  for (size_t i = 0; i < arm_.dof(); i++) {
    px[i+2] = armjpx[i];
    if (J_px_p) {
      // see compose's jacobian
      (*J_px_p)[i+2].block<6,4>(0,0) = (armjpx[i].inverse() * arm_base).AdjointMap() * Harm_base;
      (*J_px_p)[i+2].block(0,4,6,arm_.dof()) = Jarm_jpx_jp[i];
    }
  }
}

} // namespace gpmp2