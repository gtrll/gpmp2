/**
 *  @file  VehicleDynamics.h
 *  @brief vehicle dynamics functions
 *  @author Jing Dong
 *  @date  Oct 14, 2016
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>


namespace gpmp2 {


/// simple 2D vehicle dynamics: vehicle heading is consistent with velocity direction
/// return sliding velocity for Lie group, lower is better
inline double simple2DVehicleDynamicsPose2(const gtsam::Pose2& p, const gtsam::Vector3& v,
    gtsam::OptionalJacobian<1, 3> Hp = boost::none, 
    gtsam::OptionalJacobian<1, 3> Hv = boost::none) {

  if (Hp) *Hp = (gtsam::Matrix13() << 0, 0, 0).finished();
  if (Hv) *Hv = (gtsam::Matrix13() << 0, 1, 0).finished();

  return v(1);
}

/// simple 2D vehicle dynamics: vehicle heading is consistent with velocity direction
/// return sliding velocity for vector space, lower is better
inline double simple2DVehicleDynamicsVector3(const gtsam::Vector3& p, const gtsam::Vector3& v,
    gtsam::OptionalJacobian<1, 3> Hp = boost::none, 
    gtsam::OptionalJacobian<1, 3> Hv = boost::none) {

  if (Hp) *Hp = (gtsam::Matrix13() << 0, 0, -(v(1)*sin(p(2))+v(0)*cos(p(2)))).finished();
  if (Hv) *Hv = (gtsam::Matrix13() << -sin(p(2)), cos(p(2)), 0).finished();

  // sin(a-b) = sin(b)cos(a) - cos(a)sin(b)
  return v(1)*cos(p(2)) - v(0)*sin(p(2));
}

}
