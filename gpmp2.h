// gpmp2 matlab wrapper declarations

// gtsam deceleration
class gtsam::Vector6;
class gtsam::Vector3;
class gtsam::Point3;
class gtsam::Pose3;
class gtsam::Point2;
class gtsam::Pose2;

class gtsam::GaussianFactorGraph;
class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor;

namespace gpmp2 {

////////////////////////////////////////////////////////////////////////////////
// geometry
////////////////////////////////////////////////////////////////////////////////

#include <gpmp2/geometry/Pose2Vector.h>

class Pose2Vector {
  // constructors
  Pose2Vector();
  Pose2Vector(const gtsam::Pose2& pose, Vector c);
  // access
  gtsam::Pose2 pose() const;
  Vector configuration() const;
  // print
  void print(string s) const;
};


////////////////////////////////////////////////////////////////////////////////
// gp
////////////////////////////////////////////////////////////////////////////////

// prior factor
#include <gpmp2/gp/GaussianProcessPriorLinear.h>

virtual class GaussianProcessPriorLinear : gtsam::NoiseModelFactor {
  GaussianProcessPriorLinear(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};

#include <gpmp2/gp/GaussianProcessPriorPose2.h>

virtual class GaussianProcessPriorPose2 : gtsam::NoiseModelFactor {
  GaussianProcessPriorPose2(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};

#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>

virtual class GaussianProcessPriorPose2Vector : gtsam::NoiseModelFactor {
  GaussianProcessPriorPose2Vector(size_t key1, size_t key2, size_t key3, size_t key4,
      double delta, const gtsam::noiseModel::Base* Qc_model);
};


// util class for all interpolated measurements
#include <gpmp2/gp/GaussianProcessInterpolatorLinear.h>

class GaussianProcessInterpolatorLinear {
  GaussianProcessInterpolatorLinear(const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
  Vector interpolatePose(Vector pose1, Vector vel1, Vector pose2, Vector vel2) const;
  Vector interpolateVelocity(Vector pose1, Vector vel1, Vector pose2, Vector vel2) const;
};



////////////////////////////////////////////////////////////////////////////////
// kinematics
////////////////////////////////////////////////////////////////////////////////


// abstract arm class use DH params
#include <gpmp2/kinematics/Arm.h>

class Arm {
  Arm(size_t dof, Vector a, Vector alpha, Vector d);
  Arm(size_t dof, Vector a, Vector alpha, Vector d, const gtsam::Pose3& base_pose);
  Arm(size_t dof, Vector a, Vector alpha, Vector d, const gtsam::Pose3& base_pose, Vector theta_bias);
  // full forward kinematics
  Matrix forwardKinematicsPose(Vector jp) const;
  Matrix forwardKinematicsPosition(Vector jp) const;
  Matrix forwardKinematicsVel(Vector jp, Vector jv) const;
  // accesses
  size_t dof() const;
  Vector a() const;
  Vector d() const;
  Vector alpha() const;
  gtsam::Pose3 base_pose() const;
};

// abstract pose2 mobile base class
#include <gpmp2/kinematics/Pose2MobileBase.h>

class Pose2MobileBase {
  Pose2MobileBase();

  // full forward kinematics
  Matrix forwardKinematicsPose(const gtsam::Pose2& jp) const;
  Matrix forwardKinematicsPosition(const gtsam::Pose2& jp) const;
  Matrix forwardKinematicsVel(const gtsam::Pose2& jp, Vector jv) const;
  // accesses
  size_t dof() const;
  size_t nr_links() const;
};

// abstract pose2 mobile arm class
#include <gpmp2/kinematics/Pose2MobileArm.h>

class Pose2MobileArm {
  Pose2MobileArm(const gpmp2::Arm& arm);
  Pose2MobileArm(const gpmp2::Arm& arm, const gtsam::Pose3& base_T_arm);

  // full forward kinematics
  Matrix forwardKinematicsPose(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsPosition(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsVel(const gpmp2::Pose2Vector& jp, Vector jv) const;
  // accesses
  size_t dof() const;
  size_t nr_links() const;
  gpmp2::Arm arm() const;
  gtsam::Pose3 base_T_arm() const;
};

// abstract pose2 mobile with two arms class
#include <gpmp2/kinematics/Pose2Mobile2Arms.h>

class Pose2Mobile2Arms {
  Pose2Mobile2Arms(const gpmp2::Arm& arm1, const gpmp2::Arm& arm2);
  Pose2Mobile2Arms(const gpmp2::Arm& arm1, const gpmp2::Arm& arm2, 
      const gtsam::Pose3& base_T_arm1, const gtsam::Pose3& base_T_arm2);

  // full forward kinematics
  Matrix forwardKinematicsPose(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsPosition(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsVel(const gpmp2::Pose2Vector& jp, Vector jv) const;
  // accesses
  size_t dof() const;
  size_t nr_links() const;
  gpmp2::Arm arm1() const;
  gpmp2::Arm arm2() const;
  gtsam::Pose3 base_T_arm1() const;
  gtsam::Pose3 base_T_arm2() const;
};

// abstract pose2 mobile with linear actuator and one arm class
#include <gpmp2/kinematics/Pose2MobileVetLinArm.h>

class Pose2MobileVetLinArm {
  Pose2MobileVetLinArm(const gpmp2::Arm& arm);
  Pose2MobileVetLinArm(const gpmp2::Arm& arm, const gtsam::Pose3& base_T_torso, 
      const gtsam::Pose3& torso_T_arm, bool reverse_linact);

  // full forward kinematics
  Matrix forwardKinematicsPose(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsPosition(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsVel(const gpmp2::Pose2Vector& jp, Vector jv) const;
  // accesses
  size_t dof() const;
  size_t nr_links() const;
  gpmp2::Arm arm() const;
  gtsam::Pose3 base_T_torso() const;
  gtsam::Pose3 torso_T_arm() const;
  bool reverse_linact() const;
};

// abstract pose2 mobile with linear actuator and two arms class
#include <gpmp2/kinematics/Pose2MobileVetLin2Arms.h>

class Pose2MobileVetLin2Arms {
  Pose2MobileVetLin2Arms(const gpmp2::Arm& arm1, const gpmp2::Arm& arm2);
  Pose2MobileVetLin2Arms(const gpmp2::Arm& arm1, const gpmp2::Arm& arm2, 
      const gtsam::Pose3& base_T_torso,  const gtsam::Pose3& torso_T_arm1, 
      const gtsam::Pose3& torso_T_arm2, bool reverse_linact);

  // full forward kinematics
  Matrix forwardKinematicsPose(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsPosition(const gpmp2::Pose2Vector& jp) const;
  Matrix forwardKinematicsVel(const gpmp2::Pose2Vector& jp, Vector jv) const;
  // accesses
  size_t dof() const;
  size_t nr_links() const;
  gpmp2::Arm arm1() const;
  gpmp2::Arm arm2() const;
  gtsam::Pose3 base_T_torso() const;
  gtsam::Pose3 torso_T_arm1() const;
  gtsam::Pose3 torso_T_arm2() const;
  bool reverse_linact() const;
};

// Abstract Point Robot class
#include <gpmp2/kinematics/PointRobot.h>

class PointRobot {
  PointRobot(size_t dof, size_t nr_links);
  // full forward kinematics
  Matrix forwardKinematicsPose(Vector jp) const;
  Matrix forwardKinematicsPosition(Vector jp) const;
  Matrix forwardKinematicsVel(Vector jp, Vector jv) const;
  // accesses
  size_t dof() const;
  size_t nr_links() const;
};


// BodySphere class
#include <gpmp2/kinematics/RobotModel.h>

class BodySphere {
  BodySphere(size_t id, double r, const gtsam::Point3& c);
};

class BodySphereVector {
  BodySphereVector();
  void push_back(const gpmp2::BodySphere& sphere);
};


// Physical ArmModel class
#include <gpmp2/kinematics/ArmModel.h>

class ArmModel {
  ArmModel(const gpmp2::Arm& arm, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(Vector conf) const ;
  // accesses
  size_t dof() const;
  gpmp2::Arm fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};

// Physical Pose2MobileBaseModel class
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>

class Pose2MobileBaseModel {
  Pose2MobileBaseModel(const gpmp2::Pose2MobileBase& r, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(const gtsam::Pose2& conf) const ;
  // accesses
  size_t dof() const;
  gpmp2::Pose2MobileBase fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};

// Physical Pose2MobileArmModel class
#include <gpmp2/kinematics/Pose2MobileArmModel.h>

class Pose2MobileArmModel {
  Pose2MobileArmModel(const gpmp2::Pose2MobileArm& r, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(const gpmp2::Pose2Vector& conf) const ;
  // accesses
  size_t dof() const;
  gpmp2::Pose2MobileArm fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};

// Physical Pose2Mobile2ArmsModel class
#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>

class Pose2Mobile2ArmsModel {
  Pose2Mobile2ArmsModel(const gpmp2::Pose2Mobile2Arms& r, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(const gpmp2::Pose2Vector& conf) const ;
  // accesses
  size_t dof() const;
  gpmp2::Pose2Mobile2Arms fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};

// Physical Pose2MobileVetLinArmModel class
#include <gpmp2/kinematics/Pose2MobileVetLinArmModel.h>

class Pose2MobileVetLinArmModel {
  Pose2MobileVetLinArmModel(const gpmp2::Pose2MobileVetLinArm& r, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(const gpmp2::Pose2Vector& conf) const ;
  // accesses
  size_t dof() const;
  gpmp2::Pose2MobileVetLinArm fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};

// Physical Pose2MobileVetLin2ArmsModel class
#include <gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h>

class Pose2MobileVetLin2ArmsModel {
  Pose2MobileVetLin2ArmsModel(const gpmp2::Pose2MobileVetLin2Arms& r, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(const gpmp2::Pose2Vector& conf) const ;
  // accesses
  size_t dof() const;
  gpmp2::Pose2MobileVetLin2Arms fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};


// Point Robot Model class
#include <gpmp2/kinematics/PointRobotModel.h>

class PointRobotModel {
  PointRobotModel(const gpmp2::PointRobot& pR, const gpmp2::BodySphereVector& spheres);
  // solve sphere center position in world frame
  Matrix sphereCentersMat(Vector conf) const;
  // accesses
  size_t dof() const;
  gpmp2::PointRobot fk_model() const;
  size_t nr_body_spheres() const;
  double sphere_radius(size_t i) const;
};


// goal destination factor
#include <gpmp2/kinematics/GoalFactorArm.h>

virtual class GoalFactorArm : gtsam::NoiseModelFactor {
  GoalFactorArm(
      size_t poseKey, const gtsam::noiseModel::Base* cost_model,
      const gpmp2::Arm& arm, const gtsam::Point3& dest_point);
  //Vector evaluateError(Vector conf) const;      // for debug: plot error
};


// joint limit factor to vector space
#include <gpmp2/kinematics/JointLimitFactorVector.h>

virtual class JointLimitFactorVector: gtsam::NoiseModelFactor {
  JointLimitFactorVector(size_t key, const gtsam::noiseModel::Base* cost_model, 
      Vector down_limit, Vector up_limit, Vector limit_thresh);
};


// joint velocity limit factor to vector space
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>

virtual class VelocityLimitFactorVector: gtsam::NoiseModelFactor {
  VelocityLimitFactorVector(size_t key, const gtsam::noiseModel::Base* cost_model, 
      Vector vel_limit, Vector limit_thresh);
};



// Gaussian prior defined on the workspace position of a joint
#include <gpmp2/kinematics/GaussianPriorWorkspacePositionArm.h>

virtual class GaussianPriorWorkspacePositionArm : gtsam::NoiseModelFactor {
  GaussianPriorWorkspacePositionArm(
      size_t poseKey, const gpmp2::ArmModel& arm, int joint,
      const gtsam::Point3& des_position, const gtsam::noiseModel::Base* cost_model);
};

// Gaussian prior defined on the workspace orientation of a joint
#include <gpmp2/kinematics/GaussianPriorWorkspaceOrientationArm.h>

virtual class GaussianPriorWorkspaceOrientationArm : gtsam::NoiseModelFactor {
  GaussianPriorWorkspaceOrientationArm(
      size_t poseKey, const gpmp2::ArmModel& arm, int joint,
      const gtsam::Rot3& des_orientation, const gtsam::noiseModel::Base* cost_model);
};

// Gaussian prior defined on the workspace pose of a joint
#include <gpmp2/kinematics/GaussianPriorWorkspacePoseArm.h>

virtual class GaussianPriorWorkspacePoseArm : gtsam::NoiseModelFactor {
  GaussianPriorWorkspacePoseArm(
      size_t poseKey, const gpmp2::ArmModel& arm, int joint,
      const gtsam::Pose3& des_pose, const gtsam::noiseModel::Base* cost_model);
};

////////////////////////////////////////////////////////////////////////////////
// dynamics
////////////////////////////////////////////////////////////////////////////////

// dynamics factor Pose2
#include <gpmp2/dynamics/VehicleDynamicsFactorPose2.h>

virtual class VehicleDynamicsFactorPose2 : gtsam::NoiseModelFactor {
  VehicleDynamicsFactorPose2(size_t poseKey, size_t velKey, double cost_sigma);
};

// dynamics factor Pose2Vector
#include <gpmp2/dynamics/VehicleDynamicsFactorPose2Vector.h>

virtual class VehicleDynamicsFactorPose2Vector : gtsam::NoiseModelFactor {
  VehicleDynamicsFactorPose2Vector(size_t poseKey, size_t velKey, double cost_sigma);
};


// dynamics factor Vector
#include <gpmp2/dynamics/VehicleDynamicsFactorVector.h>

virtual class VehicleDynamicsFactorVector : gtsam::NoiseModelFactor {
  VehicleDynamicsFactorVector(size_t poseKey, size_t velKey, double cost_sigma);
};



////////////////////////////////////////////////////////////////////////////////
// obstacle
////////////////////////////////////////////////////////////////////////////////

// signed distance field class
#include <gpmp2/obstacle/SignedDistanceField.h>
class SignedDistanceField {
  SignedDistanceField();
  SignedDistanceField(const gtsam::Point3& origin, double cell_size, size_t field_rows,
      size_t field_cols, size_t field_z);
  // insert field data
  void initFieldData(size_t z_idx, const Matrix& field_layer);
  // access
  double getSignedDistance(const gtsam::Point3& point) const;
  void print(string s) const;
  void saveSDF(string filename);
  void loadSDF(string filename);
};


// planar signed distance field class
#include <gpmp2/obstacle/PlanarSDF.h>
class PlanarSDF {
  PlanarSDF(const gtsam::Point2& origin, double cell_size, const Matrix& data);
  // access
  double getSignedDistance(const gtsam::Point2& point) const;
  void print(string s) const;
};


// obstacle avoid factor
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
virtual class ObstacleSDFFactorArm : gtsam::NoiseModelFactor {
  ObstacleSDFFactorArm(
      size_t poseKey, const gpmp2::ArmModel& arm,
      const gpmp2::SignedDistanceField& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(Vector pose) const;
};


// obstacle avoid factor with GP interpolation
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
virtual class ObstacleSDFFactorGPArm : gtsam::NoiseModelFactor {
  ObstacleSDFFactorGPArm(
      size_t pose1Key, size_t vel1Key, size_t pose2Key, size_t vel2Key,
      const gpmp2::ArmModel& arm, const gpmp2::SignedDistanceField& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};


// Arm obstacle avoid factor (just planar arm with 2D signed distance field)
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>

virtual class ObstaclePlanarSDFFactorArm : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorArm(
      size_t posekey, const gpmp2::ArmModel& arm,
      const gpmp2::PlanarSDF& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(Vector pose) const;
};


// Arm obstacle avoid factor with GP  (just planar arm with 2D signed distance field)
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>

virtual class ObstaclePlanarSDFFactorGPArm : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorGPArm(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::ArmModel& arm, const gpmp2::PlanarSDF& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};


// planar obstacle avoid factor for Point Robot
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

virtual class ObstaclePlanarSDFFactorPointRobot : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorPointRobot(
      size_t posekey, const gpmp2::PointRobotModel& pR,
      const gpmp2::PlanarSDF& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(Vector pose) const;
};


// planar obstacle avoid factor with GP for Point Robot 
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>

virtual class ObstaclePlanarSDFFactorGPPointRobot : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorGPPointRobot(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::PointRobotModel& pR, const gpmp2::PlanarSDF& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

// planar obstacle avoid factor (pose2 mobile base with 2D signed distance field)
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h>
virtual class ObstaclePlanarSDFFactorPose2MobileBase : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorPose2MobileBase(
      size_t posekey, const gpmp2::Pose2MobileBaseModel& robot,
      const gpmp2::PlanarSDF& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gtsam::Pose2& pose) const;
};

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h>
virtual class ObstaclePlanarSDFFactorGPPose2MobileBase : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorGPPose2MobileBase(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2MobileBaseModel& robot, const gpmp2::PlanarSDF& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

// planar obstacle avoid factor (pose2 mobile arm with 2D signed distance field)
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h>
virtual class ObstaclePlanarSDFFactorPose2MobileArm : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorPose2MobileArm(
      size_t posekey, const gpmp2::Pose2MobileArmModel& marm,
      const gpmp2::PlanarSDF& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gpmp2::Pose2Vector& pose) const;
};

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h>
virtual class ObstaclePlanarSDFFactorGPPose2MobileArm : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorGPPose2MobileArm(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2MobileArmModel& marm, const gpmp2::PlanarSDF& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};


// planar obstacle avoid factor (pose2 mobile 2 arms with 2D signed distance field)
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2Mobile2Arms.h>
virtual class ObstaclePlanarSDFFactorPose2Mobile2Arms : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorPose2Mobile2Arms(
      size_t posekey, const gpmp2::Pose2Mobile2ArmsModel& marm,
      const gpmp2::PlanarSDF& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gpmp2::Pose2Vector& pose) const;
};

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2Mobile2Arms.h>
virtual class ObstaclePlanarSDFFactorGPPose2Mobile2Arms : gtsam::NoiseModelFactor {
  ObstaclePlanarSDFFactorGPPose2Mobile2Arms(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2Mobile2ArmsModel& marm, const gpmp2::PlanarSDF& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};


// obstacle avoid factor (pose2 mobile base with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h>
virtual class ObstacleSDFFactorPose2MobileBase : gtsam::NoiseModelFactor {
  ObstacleSDFFactorPose2MobileBase(
      size_t posekey, const gpmp2::Pose2MobileBaseModel& robot,
      const gpmp2::SignedDistanceField& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gtsam::Pose2& pose) const;
};

// obstacle avoid factor with GP interpolation (pose2 mobile base with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h>
virtual class ObstacleSDFFactorGPPose2MobileBase : gtsam::NoiseModelFactor {
  ObstacleSDFFactorGPPose2MobileBase(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2MobileBaseModel& robot, const gpmp2::SignedDistanceField& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

// obstacle avoid factor (pose2 mobile arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h>
virtual class ObstacleSDFFactorPose2MobileArm : gtsam::NoiseModelFactor {
  ObstacleSDFFactorPose2MobileArm(
      size_t posekey, const gpmp2::Pose2MobileArmModel& marm,
      const gpmp2::SignedDistanceField& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gpmp2::Pose2Vector& pose) const;
};

// obstacle avoid factor with GP interpolation (pose2 mobile arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h>
virtual class ObstacleSDFFactorGPPose2MobileArm : gtsam::NoiseModelFactor {
  ObstacleSDFFactorGPPose2MobileArm(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2MobileArmModel& marm, const gpmp2::SignedDistanceField& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

// obstacle avoid factor (pose2 mobile + 2 x arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h>
virtual class ObstacleSDFFactorPose2Mobile2Arms : gtsam::NoiseModelFactor {
  ObstacleSDFFactorPose2Mobile2Arms(
      size_t posekey, const gpmp2::Pose2Mobile2ArmsModel& marm,
      const gpmp2::SignedDistanceField& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gpmp2::Pose2Vector& pose) const;
};

// obstacle avoid factor with GP interpolation (pose2 mobile + 2 x arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h>
virtual class ObstacleSDFFactorGPPose2Mobile2Arms : gtsam::NoiseModelFactor {
  ObstacleSDFFactorGPPose2Mobile2Arms(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2Mobile2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

// obstacle avoid factor (pose2 mobile + linear actuator + arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h>
virtual class ObstacleSDFFactorPose2MobileVetLinArm : gtsam::NoiseModelFactor {
  ObstacleSDFFactorPose2MobileVetLinArm(
      size_t posekey, const gpmp2::Pose2MobileVetLinArmModel& marm,
      const gpmp2::SignedDistanceField& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gpmp2::Pose2Vector& pose) const;
};

// obstacle avoid factor with GP interpolation (pose2 mobile + linear actuator + arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h>
virtual class ObstacleSDFFactorGPPose2MobileVetLinArm : gtsam::NoiseModelFactor {
  ObstacleSDFFactorGPPose2MobileVetLinArm(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2MobileVetLinArmModel& marm, const gpmp2::SignedDistanceField& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

// obstacle avoid factor (pose2 mobile + linear actuator + 2 x arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h>
virtual class ObstacleSDFFactorPose2MobileVetLin2Arms : gtsam::NoiseModelFactor {
  ObstacleSDFFactorPose2MobileVetLin2Arms(
      size_t posekey, const gpmp2::Pose2MobileVetLin2ArmsModel& marm,
      const gpmp2::SignedDistanceField& sdf, double cost_sigma, double epsilon);
  Vector evaluateError(const gpmp2::Pose2Vector& pose) const;
};

// obstacle avoid factor with GP interpolation (pose2 mobile + linear actuator + 2 x arm with 3D signed distance field)
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h>
virtual class ObstacleSDFFactorGPPose2MobileVetLin2Arms : gtsam::NoiseModelFactor {
  ObstacleSDFFactorGPPose2MobileVetLin2Arms(
      size_t pose1key, size_t vel1key, size_t pose2key, size_t vel2key,
      const gpmp2::Pose2MobileVetLin2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
      double cost_sigma, double epsilon, const gtsam::noiseModel::Base* Qc_model,
      double delta_t, double tau);
};

////////////////////////////////////////////////////////////////////////////////
// planner
////////////////////////////////////////////////////////////////////////////////


/// trajectory optimizer settings
#include <gpmp2/planner/TrajOptimizerSetting.h>

class TrajOptimizerSetting {

  TrajOptimizerSetting(size_t dof);

  void set_total_step(size_t step);
  void set_total_time(double time);
  void set_flag_pos_limit(bool flag);
  void set_flag_vel_limit(bool flag);
  void set_joint_pos_limits_up(Vector v);
  void set_joint_pos_limits_down(Vector v);
  void set_vel_limits(Vector v);
  void set_pos_limit_thresh(Vector v);
  void set_vel_limit_thresh(Vector v);
  void set_pos_limit_model(Vector v);
  void set_vel_limit_model(Vector v);
  void set_epsilon(double eps);
  void set_cost_sigma(double sigma);
  void set_obs_check_inter(size_t inter);
  void set_rel_thresh(double thresh);
  void set_max_iter(size_t iter);
  void set_conf_prior_model(double sigma);
  void set_vel_prior_model(double sigma);
  void set_Qc_model(Matrix Qc);
  /// set optimization type
  void setGaussNewton();
  void setLM();
  void setDogleg();
  /// set optimization verbosity
  void setVerbosityNone();
  void setVerbosityError();
  /// set value is guaranteed not increase
  void setOptimizationNoIncrase(bool flag);
};


/// batch trajectory optimizers
#include <gpmp2/planner/BatchTrajOptimizer.h>

/// 2D arm version optimizer
gtsam::Values BatchTrajOptimize2DArm(
    const gpmp2::ArmModel& arm, const gpmp2::PlanarSDF& sdf,
    Vector start_conf, Vector start_vel, Vector end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 3D arm version optimizer
gtsam::Values BatchTrajOptimize3DArm(
    const gpmp2::ArmModel& arm, const gpmp2::SignedDistanceField& sdf,
    Vector start_conf, Vector start_vel, Vector end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 2D mobile arm version optimizer
gtsam::Values BatchTrajOptimizePose2MobileArm2D(
    const gpmp2::Pose2MobileArmModel& marm, const gpmp2::PlanarSDF& sdf,
    const gpmp2::Pose2Vector& start_conf, Vector start_vel, const gpmp2::Pose2Vector& end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile arm version optimizer
gtsam::Values BatchTrajOptimizePose2MobileArm(
    const gpmp2::Pose2MobileArmModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gpmp2::Pose2Vector& start_conf, Vector start_vel, const gpmp2::Pose2Vector& end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile + 2 x arm version optimizer
gtsam::Values BatchTrajOptimizePose2Mobile2Arms(
    const gpmp2::Pose2Mobile2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gpmp2::Pose2Vector& start_conf, Vector start_vel, const gpmp2::Pose2Vector& end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile + lin act + arm version optimizer
gtsam::Values BatchTrajOptimizePose2MobileVetLinArm(
    const gpmp2::Pose2MobileVetLinArmModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gpmp2::Pose2Vector& start_conf, Vector start_vel, const gpmp2::Pose2Vector& end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile + lin act + 2 x arm version optimizer
gtsam::Values BatchTrajOptimizePose2MobileVetLin2Arms(
    const gpmp2::Pose2MobileVetLin2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gpmp2::Pose2Vector& start_conf, Vector start_vel, const gpmp2::Pose2Vector& end_conf, Vector end_vel,
    const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);

/// 2D arm collision cost
double CollisionCost2DArm(
    const gpmp2::ArmModel& arm, const gpmp2::PlanarSDF& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 3D arm collision cost
double CollisionCost3DArm(
    const gpmp2::ArmModel& arm, const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 2D mobile base collision cost
double CollisionCostPose2MobileBase2D(
    const gpmp2::Pose2MobileBaseModel& robot, const gpmp2::PlanarSDF& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile base collision cost
double CollisionCostPose2MobileBase(
    const gpmp2::Pose2MobileBaseModel& robot, const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 2D mobile arm collision cost
double CollisionCostPose2MobileArm2D(
    const gpmp2::Pose2MobileArmModel& marm, const gpmp2::PlanarSDF& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile arm collision cost
double CollisionCostPose2MobileArm(
    const gpmp2::Pose2MobileArmModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile + 2 x arm collision cost
double CollisionCostPose2Mobile2Arms(
    const gpmp2::Pose2Mobile2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile + lin act + arm collision cost
double CollisionCostPose2MobileVetLinArm(
    const gpmp2::Pose2MobileVetLinArmModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// 3D mobile + lin act + 2 x arm collision cost
double CollisionCostPose2MobileVetLin2Arms(
    const gpmp2::Pose2MobileVetLin2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
    const gtsam::Values& result, const gpmp2::TrajOptimizerSetting& setting);

/// Optimization function for custom graphs
gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& init_values,
    const gpmp2::TrajOptimizerSetting& setting, bool iter_no_increase);

/// iSAM2 incremental trajectory optimizers
#include <gpmp2/planner/ISAM2TrajOptimizer.h>

/// 2D replanner
class ISAM2TrajOptimizer2DArm {
  ISAM2TrajOptimizer2DArm(const gpmp2::ArmModel& arm, const gpmp2::PlanarSDF& sdf,
      const gpmp2::TrajOptimizerSetting& setting);

  void initFactorGraph(Vector start_conf, Vector start_vel,
      Vector goal_conf, Vector goal_vel);
  void initValues(const gtsam::Values& init_values);
  void update();

  /// Replanning interfaces
  void changeGoalConfigAndVel(Vector goal_conf, Vector goal_vel);
  void removeGoalConfigAndVel();
  void fixConfigAndVel(size_t state_idx, Vector conf_fix, Vector vel_fix);
  void addPoseEstimate(size_t state_idx, Vector pose, Matrix pose_cov);
  void addStateEstimate(size_t state_idx, Vector pose, Matrix pose_cov, Vector vel, Matrix vel_cov);

  /// accesses
  gtsam::Values values() const;
};

/// 3D replanner
class ISAM2TrajOptimizer3DArm {
  ISAM2TrajOptimizer3DArm(const gpmp2::ArmModel& arm, const gpmp2::SignedDistanceField& sdf,
      const gpmp2::TrajOptimizerSetting& setting);

  void initFactorGraph(Vector start_conf, Vector start_vel,
      Vector goal_conf, Vector goal_vel);
  void initValues(const gtsam::Values& init_values);
  void update();

  /// Replanning interfaces
  void changeGoalConfigAndVel(Vector goal_conf, Vector goal_vel);
  void removeGoalConfigAndVel();
  void fixConfigAndVel(size_t state_idx, Vector conf_fix, Vector vel_fix);
  void addPoseEstimate(size_t state_idx, Vector pose, Matrix pose_cov);
  void addStateEstimate(size_t state_idx, Vector pose, Matrix pose_cov, Vector vel, Matrix vel_cov);

  /// accesses
  gtsam::Values values() const;
};

/// 2D mobile arm replanner
class ISAM2TrajOptimizerPose2MobileArm2D {
  ISAM2TrajOptimizerPose2MobileArm2D(const gpmp2::Pose2MobileArmModel& marm, const gpmp2::PlanarSDF& sdf,
      const gpmp2::TrajOptimizerSetting& setting);

  void initFactorGraph(const gpmp2::Pose2Vector& start_conf, Vector start_vel,
      const gpmp2::Pose2Vector& goal_conf, Vector goal_vel);
  void initValues(const gtsam::Values& init_values);
  void update();

  /// Replanning interfaces
  void changeGoalConfigAndVel(const gpmp2::Pose2Vector& goal_conf, Vector goal_vel);
  void removeGoalConfigAndVel();
  void fixConfigAndVel(size_t state_idx, const gpmp2::Pose2Vector& conf_fix, Vector vel_fix);
  void addPoseEstimate(size_t state_idx, const gpmp2::Pose2Vector& pose, Matrix pose_cov);
  void addStateEstimate(size_t state_idx, const gpmp2::Pose2Vector& pose, Matrix pose_cov, Vector vel, Matrix vel_cov);

  /// accesses
  gtsam::Values values() const;
};

/// 3D mobile arm replanner
class ISAM2TrajOptimizerPose2MobileArm {
  ISAM2TrajOptimizerPose2MobileArm(const gpmp2::Pose2MobileArmModel& marm, const gpmp2::SignedDistanceField& sdf,
      const gpmp2::TrajOptimizerSetting& setting);

  void initFactorGraph(const gpmp2::Pose2Vector& start_conf, Vector start_vel,
      const gpmp2::Pose2Vector& goal_conf, Vector goal_vel);
  void initValues(const gtsam::Values& init_values);
  void update();

  /// Replanning interfaces
  void changeGoalConfigAndVel(const gpmp2::Pose2Vector& goal_conf, Vector goal_vel);
  void removeGoalConfigAndVel();
  void fixConfigAndVel(size_t state_idx, const gpmp2::Pose2Vector& conf_fix, Vector vel_fix);
  void addPoseEstimate(size_t state_idx, const gpmp2::Pose2Vector& pose, Matrix pose_cov);
  void addStateEstimate(size_t state_idx, const gpmp2::Pose2Vector& pose, Matrix pose_cov, Vector vel, Matrix vel_cov);

  /// accesses
  gtsam::Values values() const;
};


class ISAM2TrajOptimizerPose2MobileVetLin2Arms {
  ISAM2TrajOptimizerPose2MobileVetLin2Arms(const gpmp2::Pose2MobileVetLin2ArmsModel& marm, const gpmp2::SignedDistanceField& sdf,
      const gpmp2::TrajOptimizerSetting& setting);

  void initFactorGraph(const gpmp2::Pose2Vector& start_conf, Vector start_vel,
      const gpmp2::Pose2Vector& goal_conf, Vector goal_vel);
  void initValues(const gtsam::Values& init_values);
  void update();

  /// Replanning interfaces
  void changeGoalConfigAndVel(const gpmp2::Pose2Vector& goal_conf, Vector goal_vel);
  void removeGoalConfigAndVel();
  void fixConfigAndVel(size_t state_idx, const gpmp2::Pose2Vector& conf_fix, Vector vel_fix);
  void addPoseEstimate(size_t state_idx, const gpmp2::Pose2Vector& pose, Matrix pose_cov);
  void addStateEstimate(size_t state_idx, const gpmp2::Pose2Vector& pose, Matrix pose_cov, Vector vel, Matrix vel_cov);

  /// accesses
  gtsam::Values values() const;
};


// utils for traj init and interpolation
#include <gpmp2/planner/TrajUtils.h>

/// initialization
gtsam::Values initArmTrajStraightLine(Vector init_conf, Vector end_conf, size_t total_step);
gtsam::Values initPose2VectorTrajStraightLine(const gtsam::Pose2& init_pose, Vector init_conf,
    const gtsam::Pose2& end_pose, Vector end_conf, size_t total_step);
gtsam::Values initPose2TrajStraightLine(const gtsam::Pose2& init_pose,
    const gtsam::Pose2& end_pose, size_t total_step);

/// robot arm trajectory interpolator
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step);

/// robot arm trajectory interpolator between any two states
gtsam::Values interpolateArmTraj(const gtsam::Values& opt_values,
    const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);

/// mobile arm trajectory interpolator between any two states
gtsam::Values interpolatePose2MobileArmTraj(const gtsam::Values& opt_values,
    const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);

/// mobile base trajectory interpolator between any two states
gtsam::Values interpolatePose2Traj(const gtsam::Values& opt_values,
    const gtsam::noiseModel::Base* Qc_model, double delta_t, size_t inter_step, 
    size_t start_index, size_t end_index);

////////////////////////////////////////////////////////////////////////////////
// utils
////////////////////////////////////////////////////////////////////////////////


#include <gpmp2/utils/matlabUtils.h>

/// prior factors
virtual class PriorFactorPose2Vector : gtsam::NoiseModelFactor {
  PriorFactorPose2Vector(
      size_t poseKey, const gpmp2::Pose2Vector& value,
      const gtsam::noiseModel::Base* model);
};


/// values utils
void insertPose2VectorInValues(size_t key, const gpmp2::Pose2Vector& p, gtsam::Values& values);
gpmp2::Pose2Vector atPose2VectorValues(size_t key, const gtsam::Values& values);


}
