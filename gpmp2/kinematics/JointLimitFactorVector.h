/**
 *  @file  JointLimitFactorVector.h
 *  @brief apply joint limit to vector configuration space 
 *  @author Jing Dong
 *  @date  June 12, 2017
 **/

#pragma once

#include <gpmp2/kinematics/JointLimitCost.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <iostream>
#include <vector>


namespace gpmp2 {

/**
 * unary factor to apply joint limit cost to vector configuration space
 */
class JointLimitFactorVector: public gtsam::NoiseModelFactor1<gtsam::Vector> {

private:
  // typedefs
  typedef JointLimitFactorVector This;
  typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;

  // joint limit value
  gtsam::Vector down_limit_, up_limit_;

  // hinge loss threshold
  gtsam::Vector limit_thresh_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param cost_model joint limit cost weight
   * @param limit_thresh hinge loss threshold
   */
  JointLimitFactorVector(gtsam::Key poseKey, const gtsam::SharedNoiseModel& cost_model, 
      const gtsam::Vector& down_limit, const gtsam::Vector& up_limit, const gtsam::Vector& limit_thresh) :
      Base(cost_model, poseKey), down_limit_(down_limit), up_limit_(up_limit),
      limit_thresh_(limit_thresh) {
    // check dimensions
    if ((size_t)down_limit.size() != cost_model->dim() 
        || (size_t)up_limit.size() != cost_model->dim()
        || (size_t)limit_thresh.size() != cost_model->dim())
      throw std::runtime_error("[JointLimitFactorVector] ERROR: limit vector dim does not fit.");
  }

  virtual ~JointLimitFactorVector() {}

  /// error function
  gtsam::Vector evaluateError(const gtsam::Vector& conf, 
      boost::optional<gtsam::Matrix&> H1 = boost::none) const {

    using namespace gtsam;
    if (H1)
      *H1 = Matrix::Zero(conf.size(), conf.size());
    Vector err(conf.size());
    for (size_t i = 0; i < (size_t)conf.size(); i++) {
      if (H1) {
        double Hp;
        err(i) = hingeLossJointLimitCost(conf(i), down_limit_(i), up_limit_(i), limit_thresh_(i), Hp);
        (*H1)(i, i) = Hp;
      } else {
        err(i) = hingeLossJointLimitCost(conf(i), down_limit_(i), up_limit_(i), limit_thresh_(i));
      }
    }
    return err;
  }


  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

  /** print contents */
  void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
    std::cout << s << "JointLimitFactorVector :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "Limit cost threshold : " << limit_thresh_ << std::endl;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
  }
};

}   // namespace gpmp2
