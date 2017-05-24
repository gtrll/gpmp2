/**
 *  @file  DynamicVector.h
 *  @brief Wrapper of dynamic size eigen vector with all Lie group traits implemented 
 *  @author Jing Dong
 *  @date Oct 3, 2016
 **/

#pragma once

#include <gpmp2/config.h>

#include <Eigen/Core> // from gtsam included path, use gtsam eigen version
#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorSpace.h>

#include <iostream>


namespace gpmp2 {

/**
 * wrapper of dynamic size eigen vector
 * replace raw dynamic size eigen vector, which does not have all traits
 * fully implemented in gtsam
 */
class GPMP2_EXPORT DynamicVector {

private:
  size_t dim_;
  Eigen::VectorXd vector_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // default constructor do nothing
  DynamicVector() : dim_(0) {}

  // eigen constructor
  // non-explicit conversion enabled
  DynamicVector(const Eigen::VectorXd& vector) : dim_(vector.size()), vector_(vector) {}

  virtual ~DynamicVector() {}

  // access element for convenience
  double operator()(size_t i) const { return vector_(i); }


  /// @name VectorSpace
  /// @{

  // dimension
  enum { dimension = Eigen::Dynamic };  // no static dimension implemented
  size_t dim() const { return dim_; }

  // no static identity implemented
  static DynamicVector identity() {
    throw std::runtime_error("[DynamicVector] ERROR: no static identity implemented");
    return DynamicVector();
  }

  // addition
  DynamicVector operator+(const DynamicVector& m2) const {
    return DynamicVector(this->vector_ + m2.vector_);
  }

  // subtraction
  DynamicVector operator-(const DynamicVector& m2) const {
    return DynamicVector(this->vector_ - m2.vector_);
  }

  // inversion
  DynamicVector operator-() const {
    return DynamicVector(-this->vector_);
  }

  // conversion to gtsam::Vector
  const Eigen::VectorXd& vector() const { return vector_; }

  // addition of a vector on the right
  DynamicVector operator+(const Eigen::VectorXd& vec) const {
    return DynamicVector(this->vector_ + vec);
  }

  /// @}
  /// @name Testable
  /// @{

  // print
  void print(const std::string& s) const {
    std::cout << s << vector_ << std::endl;
  }

  // equals
  bool equals(const DynamicVector& m2, double tol) const {
    return gtsam::equal_with_abs_tol(this->vector_, m2.vector_, tol);
  }

  /// @}
};

} // \ namespace gpmp2


// gtsam traits for DynamicVector
namespace gtsam {

template<>
struct traits<gpmp2::DynamicVector> : public internal::VectorSpace<gpmp2::DynamicVector> {
};

}
