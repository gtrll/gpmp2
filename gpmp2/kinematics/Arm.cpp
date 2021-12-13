/**
 *  @file  Arm.cpp
 *  @brief 3D Manipulator, Kinematics and Jacobians
 *  @author Mustafa Mukadam, Jing Dong
 *  @date  Nov 10, 2015
 **/

#include <gpmp2/kinematics/Arm.h>


using namespace gtsam;

namespace gpmp2 {

/* ************************************************************************** */
Arm::Arm(size_t dof, const Vector& a, const Vector& alpha, const Vector& d,
    const Pose3& base_pose, const Vector& theta_bias) :
    Base(dof, dof), a_(a), alpha_(alpha), d_(d), base_pose_(base_pose),
    theta_bias_(theta_bias) {

  // DH transformation for each link, without theta matrix
  // Spong06book, page. 69, eq. (3.10)
  for (size_t i = 0; i < dof; i++)
    link_trans_notheta_.push_back(
        Pose3(Rot3(), Point3(0, 0, d_(i))) *
        Pose3(Rot3(), Point3(a_(i), 0, 0)) *
        Pose3(Rot3::Rx(alpha_(i)), Point3(0, 0, 0)));
}

/* ************************************************************************** */
void Arm::forwardKinematics(
    const Vector& jp, boost::optional<const Vector&> jv,
    std::vector<gtsam::Pose3>& jpx, boost::optional<std::vector<gtsam::Vector3>&> jvx,
    boost::optional<std::vector<Matrix>&> J_jpx_jp,
    boost::optional<std::vector<Matrix>&> J_jvx_jp,
    boost::optional<std::vector<Matrix>&> J_jvx_jv) const {

  using namespace std;
  std::cout << "Line number:" << __LINE__ << std::endl;
  // space for output
  jpx.resize(dof());
  if (jvx) jvx->resize(dof());
  if (J_jpx_jp) J_jpx_jp->assign(dof(), Matrix::Zero(6, dof()));
  if (J_jvx_jp) J_jvx_jp->assign(dof(), Matrix::Zero(3, dof()));
  if (J_jvx_jv) J_jvx_jv->assign(dof(), Matrix::Zero(3, dof()));

  std::cout << "Line number:" << __LINE__ << std::endl;
  // variables
  vector<Matrix4, Eigen::aligned_allocator<Eigen::Matrix4d>> H(dof());
  vector<Matrix4, Eigen::aligned_allocator<Eigen::Matrix4d>> Ho;//(dof()); // start from 1

  vector<Matrix> J;
  if (jv) J.assign(dof(), Matrix::Zero(3, dof()));
  // vars cached for calculate output Jacobians
  vector<Matrix4, Eigen::aligned_allocator<Eigen::Matrix4d>> dH(dof());
  vector<Matrix4, Eigen::aligned_allocator<Eigen::Matrix4d>> Hoinv;//(dof()+1); // start from 1

  for (int ii=0 ; ii <dof()+1; ii++)
  {
    Ho.push_back(Matrix4::Zero());
    Hoinv.push_back(Matrix4::Zero());
  }
  std::cout << "Line number:" << __LINE__ << std::endl;
  // first iteration
  Ho[0] = base_pose_.matrix();          // H matrix for each joint, refer to origin
  Hoinv[0] = Ho[0].inverse();
  
  std::cout << "Line number:" << __LINE__ << std::endl;
  // calculate needed vars
  // indx start from 1
  for (size_t i = 1; i <= dof(); i++) {

    std::cout << "Line number:" << __LINE__ << std::endl;
    // transformation from current frame to previous frame
    H[i-1] = getH(i-1, jp(i-1));
    // tranformation from current frame to origin frame
    Ho[i] = Ho[i-1] * H[i-1];
    
    std::cout << "Line number:" << __LINE__ << std::endl;
    // velocity Jacobian from current link to origin, J = [Jv_1 ... Jv_n, 0 ... 0]
    if (jv) {
      //J[i-1].setZero();
      for (size_t j = 0; j < i; j++)
        J[i-1].col(j) = getJvj(Ho[i], Ho[j]);
    }
    
    std::cout << "Line number:" << __LINE__ << std::endl;
    // cache for jacobians
    if (J_jpx_jp) {
      dH[i-1] = getdH(i-1, jp(i-1));
      Hoinv[i] = Ho[i].inverse();
    }
  }
  std::cout << "Line number:" << __LINE__ << std::endl;
  // cache dHoi_dqj (DOF^2 memory), only fill in i >= j since others are all zeros
  vector<vector<Matrix4, Eigen::aligned_allocator<Eigen::Matrix4d>> > dHo_dq(dof(), vector<Matrix4, Eigen::aligned_allocator<Eigen::Matrix4d>>(dof()));
  if (J_jpx_jp || J_jvx_jp)
    for (size_t i = 0; i < dof(); i++)
      for (size_t j = 0; j <= i; j++)
        if (i > j){
          dHo_dq[i][j] = Ho[j] * dH[j] * Hoinv[j+1] * Ho[i+1];
          std::cout << "Line number:" << __LINE__ << std::endl;}
        else{
          dHo_dq[i][j] = Ho[j] * dH[j];
          std::cout << "Line number:" << __LINE__ << std::endl;}
  
  std::cout << "Line number:" << __LINE__ << std::endl;
  // start calculating Forward and velocity kinematics / Jacobians
  for (size_t i = 0; i < dof(); i++) {
  

    // Find end effector points
    jpx[i] = Pose3(Rot3(Ho[i+1].block<3,3>(0,0)), Point3(Ho[i+1].col(3).head<3>()));

    // and end effector velocities
    if (jv && jvx)
      (*jvx)[i] = J[i] * (*jv);

    // diff Ho[i] = Ho[i-1] * H[i] to get pJp
    if (J_jpx_jp) {
      Matrix& Jp = (*J_jpx_jp)[i];
      //Jp.setZero();         // when j > i all are zeros
      // Jp each col
      const Matrix4 inv_jpx_i = jpx[i].inverse().matrix();
      for (size_t j = 0; j <= i; j++) {
        const Matrix4 sym_se3 = inv_jpx_i * dHo_dq[i][j];
        Jp.col(j) = (Vector6() << Vector3(sym_se3(2,1), sym_se3(0,2),
            sym_se3(1,0)), sym_se3.col(3).head<3>()).finished();
      }
    }
    
   std::cout << "Line number:" << __LINE__ << std::endl;
   for (size_t kk = 0; kk <= dof(); kk++)
   {
     std::cout << "Ho:" << Ho.at(kk) << std::endl;
   }

    // diff Jvj to vJp
    if (J_jvx_jp) {
      Matrix& Jv = (*J_jvx_jp)[i];
      //Jv.setZero();
      // Jv each col <= j (j <= i)
      // Jv.col(j) = dvxi_dq.col(j) = d_Ji_qj * vi
      std::cout << "Line number:" << __LINE__ << std::endl;
      for (size_t j = 0; j <= i; j++) {
        Matrix d_Ji_qj = Matrix::Zero(3, dof());

        // for d_Ji_qj only first i cols have values
        // d_Ji_qj.col(k) = d(getJvj(i, k-1))_dqj  (k <= i)
        d_Ji_qj.col(0) = getdJvj(Ho[i+1], Ho[0], dHo_dq[i][j], Matrix4::Zero());    // k-1 < 0, use zero for dH-1
        std::cout << "Line number:" << __LINE__ << std::endl;
        for (size_t k = 1; k <= i; k++) {
          if (k-1 >= j)
            d_Ji_qj.col(k) = getdJvj(Ho[i+1], Ho[k], dHo_dq[i][j], dHo_dq[k-1][j]);
          else
            d_Ji_qj.col(k) = getdJvj(Ho[i+1], Ho[k], dHo_dq[i][j], Matrix4::Zero());    // zero dHo_dq when j > k-1
        }
        Jv.col(j) = d_Ji_qj * (*jv);
      }
    }

    std::cout << "Line number:" << __LINE__ << std::endl;
    // vJv is simply J
    if (J_jvx_jv)
      (*J_jvx_jv)[i] = J[i];
  }
}

}
