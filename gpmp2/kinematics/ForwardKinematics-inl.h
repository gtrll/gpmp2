/**
 *  @file  ForwardKinematics-inl.h
 *  @brief Abstract forward kinematics model
 *  @author Jing Dong
 *  @date  May 28, 2015
 **/

#include <gpmp2/kinematics/ForwardKinematics.h>


namespace gpmp2 {

/* ************************************************************************** */
template <class POSE, class VELOCITY>
gtsam::Matrix ForwardKinematics<POSE, VELOCITY>::forwardKinematicsPose(const Pose& jp) const {

  std::vector<gtsam::Pose3> jpx;
  forwardKinematics(jp, boost::none, jpx, boost::none);

  // convert vector in matrix
  gtsam::Matrix jpx_mat(6, nr_links_);
  for (size_t i = 0; i < nr_links_; i++)
    jpx_mat.col(i) = (gtsam::Vector6() << gtsam::Vector3(jpx[i].rotation().yaw(),
        jpx[i].rotation().pitch(), jpx[i].rotation().roll()),
        jpx[i].translation().vector()).finished();
  return jpx_mat;
}

/* ************************************************************************** */
template <class POSE, class VELOCITY>
gtsam::Matrix ForwardKinematics<POSE, VELOCITY>::forwardKinematicsPosition(const Pose& jp) const {

  std::vector<gtsam::Pose3> jpx;
  forwardKinematics(jp, boost::none, jpx, boost::none);

  // convert vector in matrix
  gtsam::Matrix jpx_mat(3, nr_links_);
  for (size_t i = 0; i < nr_links_; i++)
    jpx_mat.col(i) = jpx[i].translation().vector();
  return jpx_mat;
}

/* ************************************************************************** */
template <class POSE, class VELOCITY>
gtsam::Matrix ForwardKinematics<POSE, VELOCITY>::forwardKinematicsVel(const Pose& jp,
    const Velocity& jv) const {

  std::vector<gtsam::Pose3> jpx;
  std::vector<gtsam::Vector3> jvx;
  forwardKinematics(jp, jv, jpx, jvx);

  // convert vector in matrix
  gtsam::Matrix jpv_mat(3, nr_links_);
  for (size_t i = 0; i < nr_links_; i++)
    jpv_mat.col(i) = jvx[i];
  return jpv_mat;
}

}
