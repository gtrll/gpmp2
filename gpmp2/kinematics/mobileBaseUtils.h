/**
 *  @file  mobileBaseUtils.h
 *  @brief Some utilities for mobile robot base
 *  @author Jing Dong
 *  @date  Aug 18, 2017
 **/

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>


namespace gpmp2 {

/// convert 2D pose to 3D pose on x-y plane, with jacobian
gtsam::Pose3 computeBasePose3(const gtsam::Pose2& base_pose2,
    gtsam::OptionalJacobian<6,3> J = boost::none);

/// compute a pose3 given vehicle base pose2 and transform, with jacobian
gtsam::Pose3 computeBaseTransPose3(const gtsam::Pose2& base_pose2,
    const gtsam::Pose3& base_T_trans, 
    gtsam::OptionalJacobian<6,3> J = boost::none);

/// lift arm base in z direction, with jacobian
gtsam::Pose3 liftBasePose3(const gtsam::Pose2& base_pose2,
  double lift, const gtsam::Pose3& base_T_trans, bool reverse_linact,
  gtsam::OptionalJacobian<6,4> J = boost::none);

}
