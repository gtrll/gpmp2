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


/// simple 2D vehicle dynamics: vehicle heading is consistant as velocity direction
/// return sliding velocity, lower is better
inline double simple2DVechileDyanmics(const gtsam::Pose2& p, const gtsam::Vector3& v,
    gtsam::OptionalJacobian<1, 3> Hp = boost::none, 
    gtsam::OptionalJacobian<1, 3> Hv = boost::none) {

  if (Hp) *Hp = (gtsam::Matrix13() << 0, 0, -(v(1)*p.rotation().s()+v(0)*p.rotation().c())).finished();
  if (Hv) *Hv = (gtsam::Matrix13() << -p.rotation().s(), p.rotation().c(), 0).finished();

  // sin(a-b) = sin(b)cos(a) - cos(a)sin(b)
  return v(1)*p.rotation().c() - v(0)*p.rotation().s();
}


}

