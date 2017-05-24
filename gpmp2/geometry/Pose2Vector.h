/**
 *  @file  Pose2Vector.h
 *  @brief Lie group product of gtsam Pose2 and eigen vector
 *  @author Jing Dong
 *  @date Oct 3, 2016
 **/

#pragma once

#include <gpmp2/geometry/DynamicVector.h>
#include <gpmp2/geometry/DynamicLieTraits.h>
#include <gpmp2/geometry/ProductDynamicLieGroup.h>
#include <gpmp2/config.h>

#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/geometry/Pose2.h>

#include <iostream>

namespace gpmp2 {

/**
 * Pose2Vector is the Lie group product of gtsam Pose2 and eigen vector
 * use to describe the pose of a mobile manipulator with planner base
 */
class GPMP2_EXPORT Pose2Vector : public ProductDynamicLieGroup<gtsam::Pose2, DynamicVector> {

private:
  typedef ProductDynamicLieGroup<gtsam::Pose2, DynamicVector> Base;

public:
  // default constructor do nothing
  Pose2Vector() : Base() {}

  // constructors
  Pose2Vector(const gtsam::Pose2& p, const DynamicVector& c) : Base(p, c) {}
  Pose2Vector(const gtsam::Pose2& p, const gtsam::Vector& c) : Base(p, DynamicVector(c)) {}

  // copy constructors
  Pose2Vector(const Pose2Vector& b) : Base(b.first, b.second) {}
  Pose2Vector(const Base& b) : Base(b.first, b.second) {}

  // access
  const gtsam::Pose2& pose() const { return this->first; }
  const gtsam::Vector& configuration() const { return this->second.vector(); }

  // print
  void print(const std::string& s = "") const {
    std::cout << s;
    this->first.print("Pose: \n");
    this->second.print("Configuration: \n");
  }
};


} // \ namespace gpmp2


// gtsam traits for Pose2Vector
namespace gtsam {
template<> struct traits<gpmp2::Pose2Vector> : internal::DynamicLieGroupTraits<gpmp2::Pose2Vector> {

  static void Print(const gpmp2::Pose2Vector& m, const std::string& s = "") {
    std::cout << s;
    m.first.print("Pose: \n");
    m.second.print("Configuration: \n");
  }

  static bool Equals(const gpmp2::Pose2Vector& m1, const gpmp2::Pose2Vector& m2, double tol = 1e-8) {
    return traits<Pose2>::Equals(m1.first, m2.first, tol) &&
        traits<gpmp2::DynamicVector>::Equals(m1.second, m2.second, tol);
  }
};

}
