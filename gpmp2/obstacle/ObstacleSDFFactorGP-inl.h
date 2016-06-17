/**
 *  @file  ObstacleSDFFactorGP-inl.h
 *  @brief Obstacle avoidance cost factor, for general arm, using GP and signed distance field
 *  @author Jing Dong
 *  @date  Dec 4, 2015
 **/

#include <gpmp2/obstacle/ObstacleCost.h>

using namespace std;
using namespace gtsam;


namespace gpmp2 {

/* ************************************************************************** */
template <class ROBOT, class GPINTER>
gtsam::Vector ObstacleSDFFactorGP<ROBOT, GPINTER>::evaluateError(
    const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
    const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) const {

  const bool use_H = (H1 || H2 || H3 || H4);

  // if Jacobians used, initialize Jerr_conf as zeros
  // size: arm_nr_points_ * DOF
  Matrix Jerr_conf = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());


  // get conf by interpolation, except last pose
  typename Robot::Pose conf;
  Matrix Jconf_c1, Jconf_c2, Jconf_v1, Jconf_v2;
  if (use_H)
    conf = GPbase_.interpolatePose(conf1, vel1, conf2, vel2, Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2);
  else
    conf = GPbase_.interpolatePose(conf1, vel1, conf2, vel2);


  // run forward kinematics of this configuration
  vector<Point3> sph_centers;
  vector<Matrix> J_px_jp;
  if (H1)
    robot_.sphereCenters(conf, sph_centers, J_px_jp);
  else
    robot_.sphereCenters(conf, sph_centers);


  // allocate cost vector
  Vector err(robot_.nr_body_spheres());

  // for each point on arm stick, get error
  for (size_t sph_idx = 0; sph_idx < robot_.nr_body_spheres(); sph_idx++) {

    const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

    if (H1) {
      Matrix13 Jerr_point;
      err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf_, total_eps, Jerr_point);

      // chain rules
      Jerr_conf.row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

    } else {
      err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf_, total_eps);
    }
  }

  // update jacobians
  if (use_H)
    GPBase::updatePoseJacobians(Jerr_conf, Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2,
        H1, H2, H3, H4);

  return err;
}

}
