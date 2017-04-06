/**
 *  @file  ObstaclePlanarSDFFactor-inl.h
 *  @brief Obstacle avoidance cost factor, using planar signed distance field
 *  @author Jing Dong
 *  @date  Nov 15, 2015
 **/

#include <gpmp2/obstacle/ObstacleCost.h>

using namespace std;
using namespace gtsam;


namespace gpmp2 {

/* ************************************************************************** */
template <class ROBOT>
gtsam::Vector ObstaclePlanarSDFFactor<ROBOT>::evaluateError(
    const Pose& conf, boost::optional<gtsam::Matrix&> H1) const {

  // if Jacobians used, initialize as zeros
  // size: arm_nr_points_ * DOF
  if (H1) *H1 = Matrix::Zero(robot_.nr_body_spheres(), robot_.dof());

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
      Matrix12 Jerr_point;
      const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());
      err(sph_idx) = hingeLossObstacleCost(sph_center_2d, sdf_, total_eps, Jerr_point);

      // chain rules
      H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx].topRows<2>();

    } else {
      const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());
      err(sph_idx) = hingeLossObstacleCost(sph_center_2d, sdf_, total_eps);
    }
  }

  return err;
}

}
