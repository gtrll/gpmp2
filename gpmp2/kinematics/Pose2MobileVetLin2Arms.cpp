/**
 *  @file  Pose2MobileVetLin2Arms.cpp
 *  @brief Abstract plannar mobile manipulator, Pose2 + 2 arms on vertical linear actuator
 *  @author Jing Dong
 *  @date  Aug 20, 2016
 **/

#include <gpmp2/kinematics/Pose2MobileVetLin2Arms.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>

#include <iostream>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
Pose2MobileVetLin2Arms::Pose2MobileVetLin2Arms(const Arm& arm1, const Arm& arm2, 
    const gtsam::Pose3& base_T_torso,  const gtsam::Pose3& torso_T_arm1, 
    const gtsam::Pose3& torso_T_arm2, bool reverse_linact) :
    Base(arm1.dof() + arm2.dof() + 4, arm1.dof() + arm2.dof() + 2), 
    base_T_torso_(base_T_torso), torso_T_arm1_(torso_T_arm1), torso_T_arm2_(torso_T_arm2),
    reverse_linact_(reverse_linact), arm1_(arm1), arm2_(arm2) {

  // check arm base pose, warn if non-zero
  if (!arm1_.base_pose().equals(Pose3::identity(), 1e-6) || 
      !arm2_.base_pose().equals(Pose3::identity(), 1e-6))
    cout << "[Pose2MobileVetLin2Arms] WARNING: Arm has non-zero base pose, this base pose will be override. "
        "Set the base_T_arm in Pose2MobileArm." << endl;
}

/* ************************************************************************** */
void Pose2MobileVetLin2Arms::forwardKinematics(
    const Pose2Vector& p, boost::optional<const gtsam::Vector&> v,
    std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector3>&> vx,
    boost::optional<std::vector<gtsam::Matrix>&> J_px_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_v) const {

  if (v)
    throw runtime_error("[Pose2MobileVetLin2Arms] TODO: velocity not implemented");

  if (!v && (vx || J_vx_p || J_vx_v))
    throw runtime_error("[Pose2MobileVetLin2Arms] ERROR: only ask for velocity in workspace given velocity in "
        "configuration space");

  // space for output
  px.resize(nr_links());
  if (vx) vx->resize(nr_links());
  if (J_px_p) J_px_p->assign(nr_links(), Matrix::Zero(6, dof()));
  if (J_vx_p) J_vx_p->assign(nr_links(), Matrix::Zero(3, dof()));
  if (J_vx_v) J_vx_v->assign(nr_links(), Matrix::Zero(3, dof()));

  // vehicle & arm base pose
  Pose3 veh_base, tso_base, arm1_base, arm2_base;
  Matrix63 Hveh_base;
  Matrix64 Htso_base, Harm1_base, Harm2_base;
  if (J_px_p || J_vx_p || J_vx_v) {
    veh_base = computeBasePose3(p.pose(), Hveh_base);
    tso_base = liftBasePose3(p.pose(), p.configuration()(0), base_T_torso_, reverse_linact_, 
        Htso_base);
    Matrix6 H_tso_comp;
    arm1_base = tso_base.compose(torso_T_arm1_, H_tso_comp);
    Harm1_base = H_tso_comp * Htso_base;
    arm2_base = tso_base.compose(torso_T_arm2_, H_tso_comp);
    Harm2_base = H_tso_comp * Htso_base;

  } else {
    veh_base = computeBasePose3(p.pose());
    tso_base = liftBasePose3(p.pose(), p.configuration()(0), base_T_torso_, reverse_linact_);
    arm1_base = tso_base.compose(torso_T_arm1_);
    arm2_base = tso_base.compose(torso_T_arm2_);
  }

  // veh base link
  px[0] = veh_base;
  if (J_px_p) (*J_px_p)[0].block<6,3>(0,0) = Hveh_base;

  // torso link
  px[1] = tso_base;
  if (J_px_p) (*J_px_p)[1].block<6,4>(0,0) = Htso_base;

  // arm1 and arm2 links
  vector<Pose3> arm1jpx, arm2jpx;
  vector<Matrix> Jarm1_jpx_jp, Jarm2_jpx_jp;

  // linear actuator use first, arm1 uses next arm1.dof, then arm2 uses last arm2.dof
  arm1_.updateBasePose(arm1_base);
  arm1_.forwardKinematics(p.configuration().segment(1, arm1_.dof()), boost::none, arm1jpx, 
      boost::none, J_px_p ? boost::optional<vector<Matrix>&>(Jarm1_jpx_jp) : boost::none);
  arm2_.updateBasePose(arm2_base);
  arm2_.forwardKinematics(p.configuration().segment(1+arm1_.dof(), arm2_.dof()), boost::none, arm2jpx, 
      boost::none, J_px_p ? boost::optional<vector<Matrix>&>(Jarm2_jpx_jp) : boost::none);

  for (size_t i = 0; i < arm1_.dof(); i++) {
    px[i+2] = arm1jpx[i];
    if (J_px_p) {
      // see compose's jacobian
      (*J_px_p)[i+2].block<6,4>(0,0) = (arm1jpx[i].inverse() * arm1_base).AdjointMap() * Harm1_base;
      (*J_px_p)[i+2].block(0,4,6,arm1_.dof()) = Jarm1_jpx_jp[i];
    }
  }
  for (size_t i = 0; i < arm2_.dof(); i++) {
    px[i+2+arm1_.dof()] = arm2jpx[i];
    if (J_px_p) {
      // see compose's jacobian
      (*J_px_p)[i+2+arm1_.dof()].block<6,4>(0,0) = (arm2jpx[i].inverse() * arm2_base).AdjointMap() * Harm2_base;
      (*J_px_p)[i+2+arm1_.dof()].block(0,4+arm1_.dof(),6,arm2_.dof()) = Jarm2_jpx_jp[i];
    }
  }
}

} // namespace gpmp2

