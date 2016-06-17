/**
 *  @file  RobotModel.h
 *  @brief Abstract robot model, provide sphere-based robot model. Templated by FK model
 *  @author Jing Dong
 *  @date  May 29, 2015
 **/

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <vector>


namespace gpmp2 {

/// body sphere class, each one indicate a body sphere
struct BodySphere {
  size_t link_id;       // attched link id, 0 - nr_link-1
  double radius;        // sphere radius
  gtsam::Point3 center;        // sphere center position to the link base
  // constructor
  BodySphere(size_t id, double r, const gtsam::Point3& c) : link_id(id), radius(r), center(c) {}
  ~BodySphere() {}
};


/// vector of body sphere, typedef here to wrap in matlab
typedef std::vector<BodySphere> BodySphereVector;


/**
 * Abstract robot model, provide sphere-based robot model.
 * template parameter is forward kinematics type
 */
template <class FK>
class RobotModel {

public:
  // typedefs
  typedef FK FKModel;
  typedef typename FKModel::Pose Pose;
  typedef typename FKModel::Velocity Velocity;

private:
  FKModel fk_model_;      // fk
  BodySphereVector body_spheres_;     // body spheres

public:
  /// default contructor do nothing, for serialization
  RobotModel() : fk_model_(), body_spheres_() {}

  /// Contructor take a fk model
  RobotModel(const FKModel& fk_model, const BodySphereVector& body_spheres) :
      fk_model_(fk_model), body_spheres_(body_spheres) {}

  /// Default destructor
  virtual ~RobotModel() {}


  /// given pose in configuration space, solve sphere center vector in work space
  /// with optional associated jacobian of pose
  void sphereCenters(const Pose& jp, std::vector<gtsam::Point3>& sph_centers,
      boost::optional<std::vector<gtsam::Matrix>&> J_point_conf = boost::none) const ;

  /// given pose in configuration space, solve a single sphere center in work space
  /// for fast call of a single sphere
  /// with optional associated jacobian of pose
  gtsam::Point3 sphereCenter(size_t sph_idx, const Pose& jp,
      boost::optional<gtsam::Matrix&> J_point_conf = boost::none) const ;


  /// given pose in configuration space, solve sphere center vector in work space
  /// matlab version, no jacobians
  /// output matrix 3 * nr_sphere
  gtsam::Matrix sphereCentersMat(const Pose& jp) const ;


  /// accesses from fk_model
  const FKModel& fk_model() const { return fk_model_; }
  size_t dof() const { return fk_model_.dof(); }

  /// accesses
  size_t nr_body_spheres() const { return body_spheres_.size(); }
  size_t sphere_link_id(size_t i) const { return body_spheres_[i].link_id; }
  double sphere_radius(size_t i) const { return body_spheres_[i].radius; }
  const gtsam::Point3& sphere_center_wrt_link(size_t i) const { return body_spheres_[i].center; }
};

}

#include <gpmp2/kinematics/RobotModel-inl.h>

