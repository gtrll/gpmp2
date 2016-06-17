/**
 *  @file  RobotModel-inl.h
 *  @brief Abstract robot model, provide sphere-based robot model. Templated by FK model
 *  @author Jing Dong
 *  @date  May 29, 2015
 **/

namespace gpmp2 {

/* ************************************************************************** */
template <class FK>
void RobotModel<FK>::sphereCenters(const Pose& jp,
    std::vector<gtsam::Point3>& sph_centers,
    boost::optional<std::vector<gtsam::Matrix>&> J_point_conf) const {

  // apply fk
  std::vector<gtsam::Pose3> link_poses;
  std::vector<gtsam::Matrix> J_pose_jp;
  if (J_point_conf)
    fk_model_.forwardKinematics(jp, boost::none, link_poses, boost::none, J_pose_jp);
  else
    fk_model_.forwardKinematics(jp, boost::none, link_poses, boost::none);

  // convert to sphere centers
  sph_centers.resize(nr_body_spheres());
  if (J_point_conf) J_point_conf->resize(nr_body_spheres());

  for (size_t sph_idx = 0; sph_idx < nr_body_spheres(); sph_idx++) {
    if (J_point_conf) {
      gtsam::Matrix36 J_point_pose;
      sph_centers[sph_idx] = link_poses[body_spheres_[sph_idx].link_id].transform_from(
          body_spheres_[sph_idx].center, J_point_pose);
      (*J_point_conf)[sph_idx] = J_point_pose * J_pose_jp[body_spheres_[sph_idx].link_id];

    } else {
      sph_centers[sph_idx] = link_poses[body_spheres_[sph_idx].link_id].transform_from(
          body_spheres_[sph_idx].center);
    }
  }
}

/* ************************************************************************** */
template <class FK>
gtsam::Point3 RobotModel<FK>::sphereCenter(size_t sph_idx, const Pose& jp,
    boost::optional<gtsam::Matrix&> J_point_conf) const {

  // apply fk
  std::vector<gtsam::Pose3> link_poses;
  std::vector<gtsam::Matrix> J_pose_jp;
  if (J_point_conf)
    fk_model_.forwardKinematics(jp, boost::none, link_poses, boost::none, J_pose_jp);
  else
    fk_model_.forwardKinematics(jp, boost::none, link_poses, boost::none);

  gtsam::Point3 sph_center;
  if (J_point_conf) {
    gtsam::Matrix36 J_point_pose;
    sph_center = link_poses[body_spheres_[sph_idx].link_id].transform_from(
        body_spheres_[sph_idx].center, J_point_pose);
    *J_point_conf = J_point_pose * J_pose_jp[body_spheres_[sph_idx].link_id];

  } else {
    sph_center = link_poses[body_spheres_[sph_idx].link_id].transform_from(
        body_spheres_[sph_idx].center);
  }

  return sph_center;
}

/* ************************************************************************** */
template <class FK>
gtsam::Matrix RobotModel<FK>::sphereCentersMat(const Pose& jp) const {

  std::vector<gtsam::Point3> sph_centers;
  sphereCenters(jp, sph_centers);

  // convert to matrix
  gtsam::Matrix points_mat(3, nr_body_spheres());
  for (size_t i = 0; i < nr_body_spheres(); i++)
    points_mat.col(i) = sph_centers[i].vector();
  return points_mat;
}

}


