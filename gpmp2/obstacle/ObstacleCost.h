/**
 *  @file  ObstacleCost.h
 *  @brief obstacle cost functions, implement hinge loss function
 *  @author Jing Dong
 *  @date  May 9, 2016
 **/

#pragma once

#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/SDFexception.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <iostream>


namespace gpmp2 {


/// hinge loss obstacle cost function
inline double hingeLossObstacleCost(const gtsam::Point3& point, const SignedDistanceField& sdf,
    double eps, gtsam::OptionalJacobian<1, 3> H_point = boost::none) {

  gtsam::Vector3 field_gradient;
  double dist_signed;
  try {
    dist_signed = sdf.getSignedDistance(point, field_gradient);
  } catch (SDFQueryOutOfRange&) {
    //std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance out of range, "
    //    "assume zero obstacle cost." << std::endl;
    if (H_point) *H_point = gtsam::Matrix13::Zero();
    return 0.0;
  }

  if (dist_signed > eps) {
    // faraway no error
    if (H_point) *H_point = gtsam::Matrix13::Zero();
    return 0.0;

  } else {
    // outside but < eps or inside object
    if (H_point) *H_point = -field_gradient.transpose();
    return eps - dist_signed;
  }
}


/// hinge loss obstacle cost function, planar version
inline double hingeLossObstacleCost(const gtsam::Point2& point, const PlanarSDF& sdf,
    double eps, gtsam::OptionalJacobian<1, 2> H_point = boost::none) {

  gtsam::Vector2 field_gradient;
  double dist_signed;
  try {
    dist_signed = sdf.getSignedDistance(point, field_gradient);
  } catch (SDFQueryOutOfRange&) {
    //std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance out of range, "
    //    "assume zero obstacle cost." << std::endl;
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;
  }

  if (dist_signed > eps) {
    // faraway no error
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;

  } else {
    // outside but < eps or inside object
    if (H_point) *H_point = -field_gradient.transpose();
    return eps - dist_signed;
  }
}

}

