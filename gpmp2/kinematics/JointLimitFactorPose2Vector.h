/**
 *  @file  JointLimitFactorPose2Vector.h
 *  @brief apply joint limit to vector configuration space (R(N) part) of SE(2) x R(N)
 *  @author Jing Dong
 *  @date  Aug 20, 2017
 **/

#pragma once

#include <gpmp2/kinematics/JointLimitCost.h>
#include <gpmp2/geometry/Pose2Vector.h>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>


namespace gpmp2 {

/**
 * unary factor to apply joint limit cost to vector configuration space
 *
 * The limits are only applied to vector space part of SE(2) x R(N),
 * but the input joint limit vector and cost dim still have save dim of SE(2) x R(N) = N+3, 
 * since we would like to keep the dims consistent everywhere: factor dim reflect var dim
 * so only last N dims of limit vector and cost dims will be used
 */
class JointLimitFactorPose2Vector: public gtsam::NoiseModelFactor1<gpmp2::Pose2Vector> {

private:
  // typedefs
  typedef JointLimitFactorPose2Vector This;
  typedef gtsam::NoiseModelFactor1<gpmp2::Pose2Vector> Base;

  // joint limit value
  gtsam::Vector down_limit_, up_limit_;

  // hinge loss threshold
  gtsam::Vector limit_thresh_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor, all limit vector dim = cost_model.dim = pose.configuration.dof
   * @param cost_model joint limit cost weight
   * @param limit_thresh hinge loss threshold
   */
   JointLimitFactorPose2Vector(gtsam::Key poseKey, const gtsam::SharedNoiseModel& cost_model, 
      const gtsam::Vector& down_limit, const gtsam::Vector& up_limit, const gtsam::Vector& limit_thresh) :
      Base(cost_model, poseKey), down_limit_(down_limit), up_limit_(up_limit),
      limit_thresh_(limit_thresh) {
    // check dimensions
    if ((size_t)down_limit.size() != cost_model->dim() 
        || (size_t)up_limit.size() != cost_model->dim()
        || (size_t)limit_thresh.size() != cost_model->dim())
      throw std::runtime_error("[JointLimitFactorVector] ERROR: limit vector dim does not fit.");
  }

  virtual ~JointLimitFactorPose2Vector() {}

  /// error function
  gtsam::Vector evaluateError(const gpmp2::Pose2Vector& pose, 
      boost::optional<gtsam::Matrix&> H1 = boost::none) const {

    using namespace gtsam;
    // get configuration
    const gtsam::Vector& conf = pose.configuration();
    // error vector conf.dim + 3, first 3 dims are all zeros
    // Jacobian size = conf.dim + 3 x conf.dim + 3, first 3 col are all zeros
    if (H1)
      *H1 = Matrix::Zero(conf.size() + 3, conf.size() + 3);
    Vector err(conf.size() + 3);
    for (size_t i = 0; i < 3; i++)
      err(i) = 0.0;
    for (size_t i = 0; i < (size_t)conf.size(); i++) {
      if (H1) {
        double Hp;
        err(i+3) = hingeLossJointLimitCost(conf(i), down_limit_(i+3), up_limit_(i+3), 
            limit_thresh_(i+3), Hp);
        (*H1)(i+3, i+3) = Hp;
      } else {
        err(i+3) = hingeLossJointLimitCost(conf(i), down_limit_(i+3), up_limit_(i+3), 
            limit_thresh_(i+3));
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
    std::cout << s << "JointLimitFactorPose2Vector :" << std::endl;
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
