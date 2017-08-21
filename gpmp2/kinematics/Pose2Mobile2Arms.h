/**
 *  @file  Pose2Mobile2Arms.h
 *  @brief Plannar mobile manipulator with two arms
 *  @author Jing Dong
 *  @date  Aug 20, 2016
 **/

#pragma once

#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/ForwardKinematics.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/config.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

#include <vector>


namespace gpmp2 {

/**
 * Abstract plannar mobile manipulator with 2 arm
 * pose class is Pose2Vector, arm1 uses first arm1.dof as pose, then arm2 uses last arm2.dof
 * total DOF = 3 + arm1.dof + arm2.dof
 */
class GPMP2_EXPORT Pose2Mobile2Arms : public ForwardKinematics<Pose2Vector, gtsam::Vector> {

private:
  // typedefs
  typedef ForwardKinematics<Pose2Vector, gtsam::Vector> Base;

  // base to arm pose
  gtsam::Pose3 base_T_arm1_, base_T_arm2_;
  // arm class
  Arm arm1_, arm2_;

public:
  /// default contructor do nothing
  Pose2Mobile2Arms() {}

  /// constructor from Arm
  Pose2Mobile2Arms(const Arm& arm1, const Arm& arm2, 
      const gtsam::Pose3& base_T_arm1 = gtsam::Pose3(), 
      const gtsam::Pose3& base_T_arm2 = gtsam::Pose3());

  /// Default destructor
  virtual ~Pose2Mobile2Arms() {}


  /**
   *  Forward kinematics: joint configuration to poses in workspace
   *  Velocity kinematics: optional joint velocities to linear velocities in workspace, no anuglar rate
   *
   *  @param p position in config space
   *  @param v velocity in config space
   *  @param px link pose in work space
   *  @param vx link velocity in work space
   *  @param J_px_p et al. optional Jacobians
   **/
  void forwardKinematics(const Pose2Vector& p, boost::optional<const gtsam::Vector&> v,
      std::vector<gtsam::Pose3>& px, boost::optional<std::vector<gtsam::Vector3>&> vx,
      boost::optional<std::vector<gtsam::Matrix>&> J_px_p = boost::none,
      boost::optional<std::vector<gtsam::Matrix>&> J_vx_p = boost::none,
      boost::optional<std::vector<gtsam::Matrix>&> J_vx_v = boost::none) const;


  /// accesses
  const Arm& arm1() const { return arm1_; }
  const Arm& arm2() const { return arm2_; }
  const gtsam::Pose3& base_T_arm1() const { return base_T_arm1_; }
  const gtsam::Pose3& base_T_arm2() const { return base_T_arm2_; }
};

}
