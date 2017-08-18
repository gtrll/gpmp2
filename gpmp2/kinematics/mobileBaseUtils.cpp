/**
 *  @file  mobileBaseUtils.h
 *  @brief Some utilities for mobile robot base
 *  @author Jing Dong
 *  @date  Aug 18, 2017
 **/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
 

namespace gpmp2 {
 
/* ************************************************************************** */
gtsam::Pose3 computeBasePose3(const gtsam::Pose2& base_pose2,
    gtsam::OptionalJacobian<6,3> J = boost::none) {

  if (J) {
    J->setZero();
    const gtsam::Matrix3 Hzrot3 = gtsam::Rot3::ExpmapDerivative(
        gtsam::Vector3(0, 0, base_pose2.theta()));
    J->block<3, 1>(0, 2) = Hzrot3.col(2);
    J->block<2, 2>(3, 0) = gtsam::Matrix2::Identity();
  }
  return gtsam::Pose3(gtsam::Rot3::Rodrigues(
      gtsam::Vector3(0, 0, base_pose2.theta())),
      gtsam::Point3(base_pose2.x(), base_pose2.y(), 0.0));
}
 
/* ************************************************************************** */
gtsam::Pose3 computeArmBasePose(const gtsam::Pose2& base_pose2,
    const gtsam::Pose3& base_T_arm, 
    gtsam::OptionalJacobian<6,3> J = boost::none) {

  if (J) {
    gtsam::Matrix63 Hbasep3;
    const gtsam::Pose3 base_pose3 = computeBasePose3(base_pose2, Hbasep3);
    gtsam::Matrix6 Hcomp;
    const gtsam::Pose3 arm_base = base_pose3.compose(base_T_arm, Hcomp);
    *J = Hcomp * Hbasep3;
    return arm_base;
  } else {
    return computeBasePose3(base_pose2).compose(base_T_arm);
  }
}

}
 