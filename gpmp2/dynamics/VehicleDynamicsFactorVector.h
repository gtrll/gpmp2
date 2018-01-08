/**
 *  @file   VehicleDynamicsFactorVector.h
 *  @brief  simple 2D vehicle dynamics factor for mobile arm base in vector space
 *  @author Mustafa Mukadam
 *  @date   Jan 8, 2018
 **/


#pragma once

#include <gpmp2/dynamics/VehicleDynamics.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <iostream>
#include <vector>


namespace gpmp2 {

/**
 * unary factor for vehicle dynamics
 */
class VehicleDynamicsFactorVector: public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {

private:
  // typedefs
  typedef VehicleDynamicsFactorVector This;
  typedef gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> Base;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /* Default constructor do nothing */
  VehicleDynamicsFactorVector() {}

  /**
   * Constructor
   * @param cost_sigma cost function covariance, should to identity model
   */
  VehicleDynamicsFactorVector(gtsam::Key poseKey, gtsam::Key velKey, double cost_sigma) :
        Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey, velKey) {}

  virtual ~VehicleDynamicsFactorVector() {}


  /// error function
  /// numerical/analytic Jacobians from cost function
  gtsam::Vector evaluateError(const gtsam::Vector& conf, const gtsam::Vector& vel,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {

    using namespace gtsam;

    if (H1 || H2) {
      Matrix13 Hp, Hv;
      const double err = simple2DVehicleDynamicsVector3(conf.head<3>(),
          vel.head<3>(), Hp, Hv);
      if (H1) {
        *H1 = Matrix::Zero(1, conf.size());
        H1->block<1,3>(0,0) = Hp;
      }
      if (H2) {
        *H2 = Matrix::Zero(1, conf.size());
        H2->block<1,3>(0,0) = Hv;
      }
      return (Vector(1) << err).finished();

    } else {
      return (Vector(1) << simple2DVehicleDynamicsVector3(conf.head<3>(),
          vel.head<3>())).finished();
    }
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "VehicleDynamicsFactorVector :" << std::endl;
    Base::print("", keyFormatter);
  }


  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
  }
};

}


