/**
 *  @file  mobileBaseUtils.h
 *  @brief Some utilities for mobile robot base
 *  @author Jing Dong
 *  @date  Aug 18, 2017
 **/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>

using namespace std;
using namespace gtsam;


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
gtsam::Pose3 computeBaseTransPose3(const gtsam::Pose2& base_pose2,
    const gtsam::Pose3& base_T_trans, 
    gtsam::OptionalJacobian<6,3> J = boost::none) {

  if (J) {
    gtsam::Matrix63 Hbasep3;
    const gtsam::Pose3 base_pose3 = computeBasePose3(base_pose2, Hbasep3);
    gtsam::Matrix6 Hcomp;
    const gtsam::Pose3 arm_base = base_pose3.compose(base_T_trans, Hcomp);
    *J = Hcomp * Hbasep3;
    return arm_base;
  } else {
    return computeBasePose3(base_pose2).compose(base_T_trans);
  }
}

/* ************************************************************************** */
gtsam::Pose3 liftBasePose3(const gtsam::Pose2& base_pose2,
  double lift, const gtsam::Pose3& base_T_trans, bool reverse_linact,
  gtsam::OptionalJacobian<6,4> J = boost::none) {
  
  // a const pose to lift base
  Pose3 lift_base_pose;
  //Matrix63 Jliftbase_z;
  if (reverse_linact) {
    lift_base_pose = Pose3::Create(Rot3(), Point3(0, 0, -lift));
  } else {
    lift_base_pose = Pose3::Create(Rot3(), Point3(0, 0, lift));
  }

  if (J) {
    Matrix63 Harmbase;
    const Pose3 armbase = computeBaseTransPose3(base_pose2, base_T_trans, Harmbase);
    Matrix6 Hcomp1, Hcomp2;
    const Pose3 armbaselift = lift_base_pose.compose(armbase, Hcomp1, Hcomp2);
    // J over pose2
    J->block<6,3>(0,0) = Hcomp2 * Harmbase;
    // J over lift z
    // D_lift_base_pose = [0; I], so (Hcomp1 * D_lift_base_pose).col(2) == Hcomp1.col(5)
    //J->block<6,1>(0,3) = (Hcomp1 * Jliftbase_z).col(2);
    if (reverse_linact)
      J->block<6,1>(0,3) = -Hcomp1.col(5);
    else
      J->block<6,1>(0,3) = Hcomp1.col(5);
    return armbaselift;
  } else {
    return lift_base_pose.compose(computeBaseTransPose3(base_pose2, base_T_trans));
  }
}

}
 