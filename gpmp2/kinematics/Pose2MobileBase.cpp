/**
 *  @file   Pose2MobileBase.cpp
 *  @brief  Abstract SE(2) mobile base
 *  @author Mustafa Mukadam
 *  @date   Jan 22, 2018
 **/

#include <gpmp2/kinematics/Pose2MobileBase.h>
#include <gpmp2/kinematics/mobileBaseUtils.h>

#include <iostream>

using namespace gtsam;
using namespace std;


namespace gpmp2 {

/* ************************************************************************** */
void Pose2MobileBase::forwardKinematics(
    const gtsam::Pose2& p, boost::optional<const gtsam::Vector&> v,
    std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector3>&> vx,
    boost::optional<std::vector<gtsam::Matrix>&> J_px_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_p,
    boost::optional<std::vector<gtsam::Matrix>&> J_vx_v) const {

  if (v)
    throw runtime_error("[Pose2MobileBase] TODO: velocity not implemented");

  if (!v && (vx || J_vx_p || J_vx_v))
    throw runtime_error("[Pose2MobileBase] ERROR: only ask for velocity in workspace given velocity in "
        "configuration space");

  // allocate space
  px.resize(nr_links());
  if (vx) vx->resize(nr_links());
  if (J_px_p) J_px_p->assign(nr_links(), Matrix::Zero(6, dof()));
  if (J_vx_p) J_vx_p->assign(nr_links(), Matrix::Zero(3, dof()));
  if (J_vx_v) J_vx_v->assign(nr_links(), Matrix::Zero(3, dof()));

  // assign values
  Matrix63 Hveh_base;
  if (J_px_p || J_vx_p || J_vx_v) {
    px[0] = computeBasePose3(p, Hveh_base);
  } else {
    px[0] = computeBasePose3(p);
  }
  if (J_px_p) (*J_px_p)[0].block<6,3>(0,0) = Hveh_base;
  if (vx) {
    (*vx)[0] = Vector3((*v)[0], (*v)[1], 0.0);
    // (*J_vx_p)[0] is zero
    if (J_vx_v)
      (*J_vx_v)[0].block<2,2>(0,0) = Matrix2::Identity();
  }
}

} // namespace gpmp2
