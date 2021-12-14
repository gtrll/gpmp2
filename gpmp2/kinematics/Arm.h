/**
 *  @file  Arm.h
 *  @brief Abstract 3D Manipulator represented by DH parameters, forward kinematics and jacobians
 *  @author Mustafa Mukadam, Jing Dong
 *  @date  Nov 10, 2015
 **/

#pragma once

#include <gpmp2/kinematics/ForwardKinematics.h>
#include <gpmp2/config.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <vector>
#include <cmath>


namespace gpmp2 {

/**
 * Abstract arm class use DH parameters, without any physical model representation
 * Inherited from ForwardKinematics
 */
class GPMP2_EXPORT Arm : public ForwardKinematics<gtsam::Vector, gtsam::Vector> {

private:
  // typedefs
  typedef ForwardKinematics<gtsam::Vector, gtsam::Vector> Base;

  gtsam::Vector a_, alpha_, d_;     // raw DH parameters
  mutable gtsam::Pose3 base_pose_;  // base pose of the first link, allow change in const
  gtsam::Vector theta_bias_;        // bias of theta

  std::vector<gtsam::Pose3> link_trans_notheta_;  // transformation of each link, no theta matrix

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// default contructor do nothing
  Arm() {}

  /// Contructor take in number of joints for the arm, its DH parameters
  /// the base pose (default zero pose), and theta bias (default zero)
  Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha, const gtsam::Vector& d,
      const gtsam::Pose3& base_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0,0,0)),
      boost::optional<const gtsam::Vector&> theta_bias = boost::none);

  /// Default destructor
  virtual ~Arm() {}


  /**
   *  Forward kinematics: joint configuration to poses in workspace
   *  Velocity kinematics: optional joint velocities to linear velocities in workspace, no anuglar rate
   *
   *  @param jp joint position in config space
   *  @param jv joint velocity in config space
   *  @param jpx joint pose in work space
   *  @param jvx joint velocity in work space
   *  @param J_jpx_jp et al. optional Jacobians
   **/
  void forwardKinematics(const gtsam::Vector& jp, boost::optional<const gtsam::Vector&> jv,
      std::vector<gtsam::Pose3>& jpx, boost::optional<std::vector<gtsam::Vector3>&> jvx,
      boost::optional<std::vector<gtsam::Matrix>&> J_jpx_jp = boost::none,
      boost::optional<std::vector<gtsam::Matrix>&> J_jvx_jp = boost::none,
      boost::optional<std::vector<gtsam::Matrix>&> J_jvx_jv = boost::none) const;


  /// update base pose in const
  void updateBasePose(const gtsam::Pose3& p) const { base_pose_ = p; }


  /// accesses
  const gtsam::Vector& a() const { return a_; }
  const gtsam::Vector& d() const { return d_; }
  const gtsam::Vector& alpha() const { return alpha_; }
  const gtsam::Pose3& base_pose() const { return base_pose_; }


private:

  /// Calculate the homogenous tranformation and matrix for joint j with angle theta in the configuration space
  gtsam::Pose3 getJointTrans(size_t i, double theta) const {
    assert(i < dof());
    // DH transformation for each link, with theta matrix
    return gtsam::Pose3(gtsam::Rot3::Rz(theta + theta_bias_(i)),
        gtsam::Point3(0, 0, 0)) * link_trans_notheta_[i];
  }

  gtsam::Matrix4 getH(size_t i, double theta) const {
    return getJointTrans(i, theta).matrix();
  }

  /// dH/dtheta
  gtsam::Matrix4 getdH(size_t i, double theta) const {
    assert(i < dof());
    const double c = cos(theta + theta_bias_(i)), s = sin(theta + theta_bias_(i));
    const gtsam::Matrix4 dRot = (gtsam::Matrix4() <<
        -s, -c, 0,  0,
        c,  -s, 0,  0,
        0,  0,  0,  0,
        0,  0,  0,  0).finished();
    return dRot * link_trans_notheta_[i].matrix();
  }

  /// Calculate a single column j of the Jacobian (Jv(j)) for a given link
  gtsam::Vector3 getJvj(const gtsam::Matrix4& Hoi, const gtsam::Matrix4& Hoj) const {
    // z axis vector in the origin tranformation
    //gtsam::Matrix3 rot_z_j = gtsam::skewSymmetric(Hoj.col(2).head<3>());
    // position vector in the origin transformation
    //gtsam::Vector3 pos_o_i = Hoi.col(3).head<3>();
    //gtsam::Vector3 pos_o_j = Hoj.col(3).head<3>();
    //return rot_z_j * (pos_o_i - pos_o_j);

    return gtsam::skewSymmetric(Hoj.col(2).head<3>()) *
        (Hoi.col(3).head<3>() - Hoj.col(3).head<3>());
  }

  /// Calculate derivative of a single column j of the Jacobian (Jv(j)) for a given link
  gtsam::Vector3 getdJvj(const gtsam::Matrix4& Hoi, const gtsam::Matrix4& Hoj,
      const gtsam::Matrix4& dHoi, const gtsam::Matrix4& dHoj) const {
    //gtsam::Matrix3 rot_z_j = gtsam::skewSymmetric(Hoj.col(2).head<3>());
    //gtsam::Matrix3 drot_z_j = gtsam::skewSymmetric(dHoj.col(2).head<3>());
    //gtsam::Vector3 pos_o_i = Hoi.col(3).head<3>();
    //gtsam::Vector3 pos_o_j = Hoj.col(3).head<3>();
    //gtsam::Vector3 dpos_o_i = dHoi.col(3).head<3>();
    //gtsam::Vector3 dpos_o_j = dHoj.col(3).head<3>();
    //return rot_z_j * (dpos_o_i - dpos_o_j) + drot_z_j * (pos_o_i - pos_o_j);

    return gtsam::skewSymmetric(Hoj.col(2).head<3>()) *
        (dHoi.col(3).head<3>() - dHoj.col(3).head<3>()) +
        gtsam::skewSymmetric(dHoj.col(2).head<3>()) *
        (Hoi.col(3).head<3>() - Hoj.col(3).head<3>());
  }

};

}
