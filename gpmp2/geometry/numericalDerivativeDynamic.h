/**
 *  @file  numericalDerivativeDynamic.h
 *  @brief Dynamic size version of numericalDerivative
 *  @author  Frank Dellaert
 *  @author Jing Dong
 *  @date Oct 3, 2016
 **/

#pragma once

#include <gpmp2/geometry/utilsDynamic.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Lie.h>


namespace gpmp2 {


template<class Y, class X>
gtsam::Matrix numericalDerivativeDynamic(boost::function<Y(const X&)> h, const X& x,
    double delta = 1e-5) {

  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag,
      typename gtsam::traits<Y>::structure_category>::value),
      "Template argument Y must be a manifold type.");
  typedef gtsam::traits<Y> TraitsY;
  typedef typename TraitsY::TangentVector TangentY;

  BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag,
      typename gtsam::traits<X>::structure_category>::value),
      "Template argument X must be a manifold type.");
  static const int N = DimensionUtils<X, gtsam::traits<X>::dimension>::getDimension(x) ;
  typedef gtsam::traits<X> TraitsX;
  typedef typename TraitsX::TangentVector TangentX;

  // get value at x, and corresponding chart
  const Y hx = h(x);

  // Bit of a hack for now to find number of rows
  const TangentY zeroY = TraitsY::Local(hx, hx);
  const size_t m = zeroY.size();

  // Prepare a tangent vector to perturb x with, only works for fixed size
  TangentX dx;
  dx.setZero(N);

  // Fill in Jacobian H
  gtsam::Matrix H = gtsam::Matrix::Zero(m, N);
  const double factor = 1.0 / (2.0 * delta);
  for (int j = 0; j < N; j++) {
    dx(j) = delta;
    const TangentY dy1 = TraitsY::Local(hx, h(TraitsX::Retract(x, dx)));
    dx(j) = -delta;
    const TangentY dy2 = TraitsY::Local(hx, h(TraitsX::Retract(x, dx)));
    dx(j) = 0;
    H.col(j) << (dy1 - dy2) * factor;
  }
  return H;
}

/** use a raw C++ function pointer */
template<class Y, class X>
typename gtsam::Matrix numericalDerivativeDynamic(Y (*h)(const X&), const X& x,
    double delta = 1e-5) {
  return numericalDerivativeDynamic<Y, X>(boost::bind(h, _1), x, delta);
}

}
