/**
 *  @file   PointRobot.cpp
 *  @brief  Abstract Point Robot
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#include <gpmp2/kinematics/PointRobot.h>

using namespace gtsam;

namespace gpmp2 {

/* ************************************************************************** */
void PointRobot::forwardKinematics(
    const Vector& jp, boost::optional<const Vector&> jv,
    std::vector<gtsam::Pose3>& jpx, boost::optional<std::vector<gtsam::Vector3>&> jvx,
    boost::optional<std::vector<Matrix>&> J_jpx_jp,
    boost::optional<std::vector<Matrix>&> J_jvx_jp,
    boost::optional<std::vector<Matrix>&> J_jvx_jv) const {

  // space for output
  jpx.resize(nr_links());
  if (jvx) jvx->resize(nr_links());
  if (J_jpx_jp) J_jpx_jp->assign(nr_links(), Matrix::Zero(6, dof()));
  if (J_jvx_jp) J_jvx_jp->assign(nr_links(), Matrix::Zero(3, dof()));
  if (J_jvx_jv) J_jvx_jv->assign(nr_links(), Matrix::Zero(3, dof()));

  Matrix H1, H2;

  // start calculating Forward and velocity kinematics and Jacobians
  for (size_t i = 0; i < nr_links(); i++) {
    // pose in workspace
    jpx[i] = Pose3::Create(Rot3(), Point3(jp[0],jp[1],0), H1, H2);

    // velocity in workspace
    if (jv && jvx)
      (*jvx)[i] << (*jv)[0], (*jv)[1], 0;

    // Jacobians
    if (J_jpx_jp) {
      (*J_jpx_jp)[i].col(0) = H2.col(0);
      (*J_jpx_jp)[i].col(1) = H2.col(1);
    }
    if (J_jvx_jv) {
      (*J_jvx_jv)[i].block<2,2>(0, 0) = Matrix::Identity(2,2);
    }
  }
}

} // namespace gpmp2
