/**
 *  @file  JointLimitCost.h
 *  @brief apply joint limit cost 
 *  @author Jing Dong
 *  @date  June 12, 2017
 **/

#pragma once

#include <boost/optional.hpp>


namespace gpmp2 {

/// hinge loss joint limit cost function
inline double hingeLossJointLimitCost(double p, double down_limit, double up_limit,
    double thresh, boost::optional<double&> H_p = boost::none) {

  if (p < down_limit + thresh) {
    if (H_p) *H_p = -1.0;
    return down_limit + thresh - p;
  
  } else if (p <= up_limit - thresh) {
    if (H_p) *H_p = 0.0;
    return 0.0;
  
  } else {
    if (H_p) *H_p = 1.0;
    return p - up_limit + thresh;
  }
}

}   // namespace gpmp2
