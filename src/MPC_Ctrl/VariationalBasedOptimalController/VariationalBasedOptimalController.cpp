//
// Created by ray on 12/12/23.
//

#include "VariationalBasedOptimalController.h"

#include <iostream>
using namespace std;

VariationalBasedOptimalController::VariationalBasedOptimalController()
    : QProblemObj_qpOASES_(VBOC_NUM_VARIABLES_QP, VBOC_NUM_CONSTRAINTS_QP) {
  // *!! It seems like this next block of 5 lines does not actually do anything,
  // I had to set print level in the OptimizationThread class
  Options options;
  options.printLevel = PL_NONE;
  QProblemObj_qpOASES_.setOptions(options);
  QProblemObj_qpOASES_.setPrintLevel(PL_NONE);

  QPFinished_ = false;

  // Eigen QP matrices
  H_eigen_.resize(VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP);
  A_eigen_.resize(VBOC_NUM_CONSTRAINTS_QP, VBOC_NUM_VARIABLES_QP);
  g_eigen_.resize(VBOC_NUM_VARIABLES_QP, 1);
  xOpt_eigen_.resize(VBOC_NUM_VARIABLES_QP, 1);
  xOpt_combined.resize(VBOC_NUM_VARIABLES_QP, 1);
  yOpt_eigen_.resize(VBOC_NUM_VARIABLES_QP + VBOC_NUM_CONSTRAINTS_QP, 1);

  mass_ = 41;
  inertia_ = 0.01;

  /* Model and World parameters and force limits */
  Ig_.resize(3, 3);
  Ig_ << .35, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
  gravity_.resize(3, 1);

  direction_normal_flat_ground_.resize(3, 1);
  direction_tangential_flat_ground_.resize(3, 1);

  /* Initialize to all feet on the ground */
  contact_state_.resize(4, 1);
  contact_state_ << 1, 1, 1, 1;

  minNormalForces_feet_.resize(4, 1);
  maxNormalForces_feet_.resize(4, 1);

  /* Actual Kinematics*/
  x_COM_world_.resize(3, 1);
  xdot_COM_world_.resize(3, 1);
  omega_b_world_.resize(3, 1);
  omega_b_body_.resize(3, 1);
  quat_b_world_.resize(4, 1);
  R_b_world_.resize(3, 3);
  p_feet_.resize(3, 4);

  /* Desired Kinematics */
  x_COM_world_desired_.resize(3, 1);
  xdot_COM_world_desired_.resize(3, 1);
  xddot_COM_world_desired_.resize(3, 1);
  omega_b_world_desired_.resize(3, 1);
  omega_b_body_desired_.resize(3, 1);
  omega_dot_b_world_desired_.resize(3, 1);
  R_b_world_desired_.resize(3, 3);
  orientation_error_.resize(3, 1);
  p_feet_desired_.resize(3, 4);

  /* Error Calculations */
  error_x_lin_.setZero(3, 1);
  error_dx_lin_.setZero(3, 1);
  error_R_lin_.setZero(3, 1);
  error_omega_lin_.setZero(3, 1);

  vbd_command_eigen_.resize(3, 1);
  vbd_command_eigen_.setZero();

  /* Temporary, Internal Matrices */
  omegaHat_.resize(3, 3);
  tempSkewMatrix3_.resize(3, 3);
  tempVector3_.resize(3, 1);

  tempSkewMatrix4_.resize(3, 3);

  tempSkewMatrix3_.setZero();
  tempVector3_.setZero();

  tempSkewMatrix4_.setZero();

  /* Robot Control variables for QP Matrices */
  f_unc_.resize(12, 1);
  C_control_.resize(VBOC_NUM_CONSTRAINTS_QP, 3 * VBOC_NUM_CONTACT_POINTS);
  C_times_f_opt_.resize(VBOC_NUM_CONSTRAINTS_QP, 1);
  C_control_.setZero();

  /* Robot Control variables used in LQR for QP Optimization */
  A_LQR_.resize(12, 12);
  A_LQR_.setZero();
  B_LQR_.resize(12, 3 * VBOC_NUM_CONTACT_POINTS);
  B_LQR_.setZero();
  B_QP_.resize(12, 12);
  B_QP_.setZero();
  P_LQR_.resize(12, 12);
  P_LQR_.setZero();
  f_ref_world_.resize(3 * VBOC_NUM_CONTACT_POINTS, 1);
  Q1_LQR_.resize(12, 12);
  Q1_LQR_.setZero();
  R1_QP_.resize(3 * VBOC_NUM_CONTACT_POINTS, 3 * VBOC_NUM_CONTACT_POINTS);
  R1_QP_.setZero();
  R2_QP_.resize(3 * VBOC_NUM_CONTACT_POINTS, 3 * VBOC_NUM_CONTACT_POINTS);
  R2_QP_.setZero();
  s_LQR_.resize(12, 1);
  H_LQR_.resize(2 * VBOC_NUM_VARIABLES_QP, 2 * VBOC_NUM_VARIABLES_QP);

  /* Initialize Optimization variables */
  xOptPrev_.setZero(12);
  yOptPrev_.setZero(VBOC_NUM_VARIABLES_QP + VBOC_NUM_CONSTRAINTS_QP);
  for (int i = 0; i < VBOC_NUM_VARIABLES_QP; i++) {
    xOpt_qpOASES_[i] = 0.0;
    xOptPrev_[i] = 0.0;
    xOpt_initialGuess_[i] = 0.0;
  }

  for (int i = 0; i < 4; i++) {
    xOpt_initialGuess_[3 * i + 2] = 20;
    xOptPrev_[3 * i + 2] = 20;
  }

  for (int i = 0; i < VBOC_NUM_VARIABLES_QP + VBOC_NUM_CONSTRAINTS_QP; i++) {
    yOpt_qpOASES_[i] = 0.0;
  }

  //  set_RobotLimits();
  setWorldData();

  cpu_time_ = 0.001;
  cpu_time_fixed_ = 0.001;
  qp_exit_flag_ = -1.0;

  qp_not_init_ = 1.0;
}

/* --------------- Primary Interface ------------------- */

void VariationalBasedOptimalController::updateProblemData(double* state_x_feedback_in, double* p_feet_in,
                                                          double* p_feet_desired_in, double* /*rpy_des_in*/, double* /*rpy_act_in*/) {
  // Unpack inputs
  copy_Array_to_Eigen(quat_b_world_, state_x_feedback_in, 4, 0);
  copy_Array_to_Eigen(x_COM_world_, state_x_feedback_in, 3, 4);
  copy_Array_to_Eigen(omega_b_world_, state_x_feedback_in, 3, 7);
  copy_Array_to_Eigen(xdot_COM_world_, state_x_feedback_in, 3, 10);
  copy_Array_to_Eigen(p_feet_, p_feet_in, 12, 0);
  copy_Array_to_Eigen(p_feet_desired_, p_feet_desired_in, 12, 0);

  // Rotation matrix from quaternions
  quaternion_to_rotationMatrix(R_b_world_, quat_b_world_);

  // Express body angular velocity in body frame
  omega_b_body_ = R_b_world_.transpose() * omega_b_world_;
  omega_b_body_desired_ = R_b_world_desired_.transpose() * omega_b_world_desired_;

  // Calculate linearized error
  calc_linear_error();

  // Calculate matrices for linearized dynamics sdot = A*s+B*df
  update_A_LQR();
  update_B_LQR();

  // Solve Continuous-time Algebraic Ricatti Equation
  update_P_LQR();

  // Compute QP Problem data
  calc_H_qpOASES();
  calc_A_qpOASES();
  calc_g_qpOASES();

  cpu_time_ = cpu_time_fixed_;
  nWSR_qpOASES_ = nWSR_fixed_;
}

void VariationalBasedOptimalController::updateContactData(double* contact_state_in, double* min_forces_in, double* max_forces_in,
                                                          double threshold_in, int stance_legs_in) {
  if (stance_legs_in == 5) {
    max_forces_in[0] = 10;
    max_forces_in[3] = 10;
  }

  // Unpack inputs
  copy_Array_to_Eigen(contact_state_, contact_state_in, 4, 0);
  copy_Array_to_Eigen(minNormalForces_feet_, min_forces_in, 4, 0);
  copy_Array_to_Eigen(maxNormalForces_feet_, max_forces_in, 4, 0);
  threshold_ = threshold_in;
  stance_legs_ = stance_legs_in;

  calc_lb_ub_qpOASES();
  calc_lbA_ubA_qpOASES();
}

void VariationalBasedOptimalController::solveQP(double* xOpt) {
  copy_real_t_to_Eigen(xOpt_eigen_, xOpt_qpOASES_, 12);

  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    tempVector3_ = -R_b_world_.transpose() * xOpt_eigen_.segment(3 * i, 3);
    xOpt[3 * i] = tempVector3_(0);
    xOpt[3 * i + 1] = tempVector3_(1);
    xOpt[3 * i + 2] = tempVector3_(2);
  }

  calc_constraint_check();
}

void VariationalBasedOptimalController::solveQP_nonThreaded(double* xOpt) {
  // &cpu_time
  if (qp_not_init_ == 1.0) {
    qp_exit_flag_ = QProblemObj_qpOASES_.init(H_qpOASES_, g_qpOASES_, A_qpOASES_, lb_qpOASES_, ub_qpOASES_, lbA_qpOASES_,
                                              ubA_qpOASES_, nWSR_qpOASES_, &cpu_time_, xOpt_initialGuess_);
    qp_not_init_ = 0.0;

    nWSR_initial_ = nWSR_qpOASES_;
    cpu_time_initial_ = cpu_time_;
  } else {
    qp_exit_flag_ =
        QProblemObj_qpOASES_.init(H_qpOASES_, g_qpOASES_, A_qpOASES_, lb_qpOASES_, ub_qpOASES_, lbA_qpOASES_, ubA_qpOASES_,
                                  nWSR_qpOASES_, &cpu_time_, xOpt_qpOASES_, yOpt_qpOASES_, &guessedBounds_, &guessedConstraints_);
  }

  QProblemObj_qpOASES_.getPrimalSolution(xOpt_qpOASES_);
  QProblemObj_qpOASES_.getDualSolution(yOpt_qpOASES_);

  QProblemObj_qpOASES_.getBounds(guessedBounds_);
  QProblemObj_qpOASES_.getConstraints(guessedConstraints_);

  copy_real_t_to_Eigen(xOpt_eigen_, xOpt_qpOASES_, 12);

  // Combine reference control (f_des) and linear control (xOpt_eigen) inputs
  xOpt_combined = f_ref_world_ + xOpt_eigen_;
  xOptPrev_ = xOpt_eigen_;

  // Transform forces into body coordinates
  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    tempVector3_ = -R_b_world_.transpose() * xOpt_combined.segment(3 * i, 3);
    xOpt[3 * i] = tempVector3_(0);
    xOpt[3 * i + 1] = tempVector3_(1);
    xOpt[3 * i + 2] = tempVector3_(2);
  }

  QProblemObj_qpOASES_.reset();

  calc_constraint_check();
//  two_contact_stand_data_publish_ = two_contact_stand_data_;
}

/* --------------- Control Math ------------ */

void VariationalBasedOptimalController::calc_linear_error() {
  // Linear error for LQR
  error_x_lin_ = x_COM_world_ - x_COM_world_desired_;
  error_dx_lin_ = xdot_COM_world_ - xdot_COM_world_desired_;
  inverseCrossMatrix(0.5 * (R_b_world_desired_.transpose() * R_b_world_ - R_b_world_.transpose() * R_b_world_desired_),
                     error_R_lin_);
  error_omega_lin_ = omega_b_body_ - R_b_world_.transpose() * R_b_world_desired_ * omega_b_body_desired_;
  s_LQR_ << error_x_lin_(0), error_x_lin_(1), error_x_lin_(2), error_dx_lin_(0), error_dx_lin_(1), error_dx_lin_(2),
      error_R_lin_(0), error_R_lin_(1), error_R_lin_(2), error_omega_lin_(0), error_omega_lin_(1), error_omega_lin_(2);

  // Orientation error for data logging purposes
  matrixLogRot(R_b_world_desired_ * R_b_world_.transpose(), orientation_error_);
}

void VariationalBasedOptimalController::calc_constraint_check() {
  C_times_f_opt_ = C_control_ * xOpt_eigen_;
}

/* --------------- QP Matrices and Problem Data ------------ */

void VariationalBasedOptimalController::update_A_LQR() {
  // Temporary variables for block assignment
  Eigen::MatrixXd tempBlock;
  tempBlock.setZero(3, 3);
  Eigen::VectorXd rd;
  rd.resize(3, 1);
  Eigen::MatrixXd rd_hat;
  rd_hat.resize(3, 3);

  // Update the A matrix in sdot = A*s+B*df
  tempSkewMatrix3_.setIdentity();
  A_LQR_.block<3, 3>(0, 3) << tempSkewMatrix3_;
  A_LQR_.block<3, 3>(6, 9) << tempSkewMatrix3_;
  crossMatrix(tempSkewMatrix3_, -omega_b_body_desired_);
  A_LQR_.block<3, 3>(6, 6) << tempSkewMatrix3_;

  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    tempVector3_ << f_ref_world_(3 * i), f_ref_world_(3 * i + 1), f_ref_world_(3 * i + 2);
    crossMatrix(tempSkewMatrix3_, tempVector3_);
    tempBlock << tempBlock + Ig_.inverse() * R_b_world_desired_.transpose() * tempSkewMatrix3_;
  }
  A_LQR_.block<3, 3>(9, 0) << tempBlock;

  tempBlock.setZero();
  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    tempVector3_ << f_ref_world_(3 * i), f_ref_world_(3 * i + 1), f_ref_world_(3 * i + 2);
    rd << p_feet_desired_.col(i);
    crossMatrix(rd_hat, rd);
    crossMatrix(tempSkewMatrix3_, rd_hat * tempVector3_);
    tempBlock << tempBlock + Ig_.inverse() * R_b_world_desired_.transpose() * tempSkewMatrix3_;
  }
  A_LQR_.block<3, 3>(9, 6) << tempBlock;

  /* NOTE: WE ASSUME THAT DESIRED ANGULAR VELOCITY IS ZERO
  if desired angular velocity of the body is nonzero, an additional block needs to be
  added at A_LQR.block<3,3>(9,9) too account for Coriolis term */
  tempBlock.setZero();
  crossMatrix(tempSkewMatrix3_, Ig_ * omega_b_body_desired_);
  crossMatrix(tempSkewMatrix4_, omega_b_body_desired_);
  tempBlock << Ig_.inverse() * (tempSkewMatrix3_ - tempSkewMatrix4_ * Ig_);
  A_LQR_.block<3, 3>(9, 9) << tempBlock;
}

void VariationalBasedOptimalController::update_B_LQR() {
  // Determine size of B based on number of legs on the ground
  contact_legs_ = 0;
  for (int leg = 0; leg < 4; leg++) {
    if (contact_state_(leg) > threshold_) contact_legs_++;
  }
  B_LQR_.resize(VBOC_NUM_VARIABLES_QP, 3 * contact_legs_);
  B_LQR_.setZero();
  R_LQR_.resize(3 * contact_legs_, 3 * contact_legs_);
  R_LQR_.setZero();

  // Build B matrix accordingly
  leg_cnt_ = 0;
  for (int leg = 0; leg < 4; leg++) {
    if (stance_legs_ == 5 && (leg == 0 || leg == 3)) {  // zero B columns for FR and BL legs
      tempSkewMatrix3_.setZero();
      B_LQR_.block<3, 3>(3, 3 * leg_cnt_) << tempSkewMatrix3_;
      B_LQR_.block<3, 3>(9, 3 * leg_cnt_) << tempSkewMatrix3_;
      leg_cnt_++;
    } else if (contact_state_(leg) > threshold_) {  // Compute B_LQR using only legs in contact
      Eigen::VectorXd rd;
      rd.resize(3, 1);
      tempSkewMatrix3_.setIdentity();
      B_LQR_.block<3, 3>(3, 3 * leg_cnt_) << 1 / mass_ * tempSkewMatrix3_;
      rd << p_feet_desired_.col(leg);
      crossMatrix(tempSkewMatrix3_, rd);
      B_LQR_.block<3, 3>(9, 3 * leg_cnt_) << Ig_.inverse() * R_b_world_desired_.transpose() * tempSkewMatrix3_;
      leg_cnt_++;
    }

    // Set B for QP - invariant to contact configuation
    Eigen::VectorXd rd;
    rd.resize(3, 1);
    tempSkewMatrix3_.setIdentity();
    B_QP_.block<3, 3>(3, 3 * leg) << 1 / mass_ * tempSkewMatrix3_;
    rd << p_feet_desired_.col(leg);
    crossMatrix(tempSkewMatrix3_, rd);
    B_QP_.block<3, 3>(9, 3 * leg) << Ig_.inverse() * R_b_world_desired_.transpose() * tempSkewMatrix3_;
  }

  // Build weight matrix R to match dimension
  for (int i = 0; i < 3 * contact_legs_; i++) R_LQR_(i, i) = R1_QP_(i, i);

  if (stance_legs_ == 5) {  // Increase cost for out of contact legs
    for (int i = 0; i < 3; i++) {
      R_LQR_(i, i) = 20 * R_LQR_(i, i);
      R_LQR_(i + 9, i + 9) = 20 * R_LQR_(i + 9, i + 9);
    }
  }
}

void VariationalBasedOptimalController::update_P_LQR() {
  /* Solve the continuous time algebraic Ricatti equation via the Schur method */

  // Compute the Hamiltonian and its eigenvalues/vectors
  H_LQR_.block<VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP>(0, 0) << A_LQR_;
  H_LQR_.block<VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP>(0, VBOC_NUM_VARIABLES_QP)
      << B_LQR_ * R_LQR_.inverse() * B_LQR_.transpose();
  H_LQR_.block<VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP>(VBOC_NUM_VARIABLES_QP, 0) << Q1_LQR_;
  H_LQR_.block<VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP>(VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP) << -A_LQR_.transpose();
  EigenSolver<MatrixXd> es(H_LQR_);

  // Create a 2nxn matrix U=[U11;U21] containing the eigenvectors of the stable eigenvalues
  Eigen::MatrixXcd U;
  U.setZero(2 * VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP);
  Eigen::MatrixXcd U11;
  U11.setZero(VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP);
  Eigen::MatrixXcd U21;
  U21.setZero(VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP);
  Eigen::MatrixXcd P_complex;
  P_complex.setZero(VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP);
  std::complex<double> lambda;

  // U contains eigenvectors corresponding to stable eigenvalues of the Hamiltonian
  int j = 0;
  for (int i = 0; i < 2 * VBOC_NUM_VARIABLES_QP; i++) {
    lambda = es.eigenvalues()[i];
    if (lambda.real() < 0) {
      U.block<2 * VBOC_NUM_VARIABLES_QP, 1>(0, j) << es.eigenvectors().col(i);
      j = j + 1;
    }
  }

  // Compute P based on U11*P = -U21;
  U11 = U.block<VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP>(0, 0);
  U21 = U.block<VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP>(VBOC_NUM_VARIABLES_QP, 0);
  P_complex = -U21 * U11.inverse();
  P_LQR_ = P_complex.real();

  // Optimal control policy
  f_unc_ = -R1_QP_.inverse() * B_QP_.transpose() * P_LQR_ * s_LQR_;
  cost_to_go_ = s_LQR_.transpose() * P_LQR_ * s_LQR_;
}

void VariationalBasedOptimalController::calc_H_qpOASES() {
  // Use LQR matrices to compute the QP cost matrix H
  // H_eigen = 2*(R1_QP);
  H_eigen_ = 2 * (R1_QP_ + R2_QP_);

  // Copy to real_t array (qpOASES data type)
  copy_Eigen_to_real_t(H_qpOASES_, H_eigen_, VBOC_NUM_VARIABLES_QP, VBOC_NUM_VARIABLES_QP);
}

void VariationalBasedOptimalController::calc_A_qpOASES() {
  Eigen::Vector3d t1x;
  t1x << 1, 0, 0;
  Eigen::Vector3d t2y;
  t2y << 0, 1, 0;

  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    C_control_.block<1, 3>(5 * i + 0, 3 * i)
        << -mu_friction_ * .7071 * direction_normal_flat_ground_.transpose() + t1x.transpose();
    C_control_.block<1, 3>(5 * i + 1, 3 * i)
        << -mu_friction_ * .7071 * direction_normal_flat_ground_.transpose() + t2y.transpose();
    C_control_.block<1, 3>(5 * i + 2, 3 * i)
        << mu_friction_ * .7071 * direction_normal_flat_ground_.transpose() + t2y.transpose();
    C_control_.block<1, 3>(5 * i + 3, 3 * i)
        << mu_friction_ * .7071 * direction_normal_flat_ground_.transpose() + t1x.transpose();
    C_control_.block<1, 3>(5 * i + 4, 3 * i) << direction_normal_flat_ground_.transpose();
  }

  copy_Eigen_to_real_t(A_qpOASES_, C_control_, VBOC_NUM_CONSTRAINTS_QP, VBOC_NUM_VARIABLES_QP);
}

void VariationalBasedOptimalController::calc_g_qpOASES() {
  // Use LQR matrices to compute the QP cost vector g
  // g_eigen = 2*(B_QP.transpose()*P_LQR.transpose()*s_LQR);
  g_eigen_ = 2 * (B_QP_.transpose() * P_LQR_.transpose() * s_LQR_ - R2_QP_ * xOptPrev_);

  // Copy to real_t array (qpOASES data type)
  copy_Eigen_to_real_t(g_qpOASES_, g_eigen_, VBOC_NUM_VARIABLES_QP, 1);
}

void VariationalBasedOptimalController::calc_lb_ub_qpOASES() {
  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    for (int j = 0; j < VBOC_NUM_VARIABLES_PER_FOOT; j++) {
      lb_qpOASES_[VBOC_NUM_VARIABLES_PER_FOOT * i + j] = contact_state_(i) * VBOC_NEGATIVE_NUMBER;
      ub_qpOASES_[VBOC_NUM_VARIABLES_PER_FOOT * i + j] = contact_state_(i) * VBOC_POSITIVE_NUMBER;
    }
  }
}

void VariationalBasedOptimalController::calc_lbA_ubA_qpOASES() {
  for (int i = 0; i < VBOC_NUM_CONTACT_POINTS; i++) {
    lbA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i] = contact_state_(i) * VBOC_NEGATIVE_NUMBER;
    lbA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 1] = contact_state_(i) * VBOC_NEGATIVE_NUMBER;
    lbA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 2] = -mu_friction_ * f_ref_world_(3 * i + 2) * .7071;
    lbA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 3] = -mu_friction_ * f_ref_world_(3 * i + 2) * .7071;
    lbA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 4] = contact_state_(i) * minNormalForces_feet_(i) - f_ref_world_(3 * i + 2);

    ubA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i] = mu_friction_ * f_ref_world_(3 * i + 2) * .7071;
    ubA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 1] = mu_friction_ * f_ref_world_(3 * i + 2) * .7071;
    ubA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 2] = contact_state_(i) * VBOC_POSITIVE_NUMBER;
    ubA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 3] = contact_state_(i) * VBOC_POSITIVE_NUMBER;
    ubA_qpOASES_[VBOC_NUM_CONSTRAINTS_PER_FOOT * i + 4] = contact_state_(i) * maxNormalForces_feet_(i) - f_ref_world_(3 * i + 2);
  }
}


/* ------------ Update Reference States Values -------------- */

void VariationalBasedOptimalController::updateDesiredTrajectoryData(double* rpy_des_in, double* p_des_in, double* omega_des_in,
                                                                    double* v_des_in) {
  x_COM_world_desired_ << p_des_in[0], p_des_in[1], p_des_in[2];
  rpyToR(R_b_world_desired_, rpy_des_in);
  omega_b_world_desired_ << omega_des_in[0], omega_des_in[1], omega_des_in[2];
  xdot_COM_world_desired_ << v_des_in[0], v_des_in[1], v_des_in[2];
}

void VariationalBasedOptimalController::updateReference_GRF(double* f_ref_in) {
  copy_Array_to_Eigen(f_ref_world_, f_ref_in, 12, 0);
}

/* ------------ Set Parameter Values -------------- */

void VariationalBasedOptimalController::setVBOCSolverConfig(double* x_weights_in, double* xdot_weights_in, double* R_weights_in,
                                                            double* omega_weights_in, double alpha_control_in,
                                                            double beta_control_in, double mu_in, double mass_in, double Ixx,
                                                            double Iyy, double Izz, double min_force, double max_force) {
  setRobotLimits(min_force, max_force);
  setLQRWeights(x_weights_in, xdot_weights_in, R_weights_in, omega_weights_in, alpha_control_in, beta_control_in);
  setFriction(mu_in);
  setMass(mass_in);
  setInertia(Ixx, Iyy, Izz);
}

void VariationalBasedOptimalController::setLQRWeights(double* x_weights_in, double* xdot_weights_in, double* R_weights_in,
                                                      double* omega_weights_in, double alpha_control_in, double beta_control_in) {
  Q1_LQR_.setIdentity();
  R1_QP_.setIdentity();
  R2_QP_.setIdentity();

  for (int i = 0; i < 3; i++) {
    Q1_LQR_(i, i) = x_weights_in[i];
    Q1_LQR_(i + 3, i + 3) = xdot_weights_in[i];
    Q1_LQR_(i + 6, i + 6) = R_weights_in[i];
    Q1_LQR_(i + 9, i + 9) = omega_weights_in[i];
  }

  for (int i = 0; i < 3 * VBOC_NUM_CONTACT_POINTS; i++) {
    R1_QP_(i, i) = alpha_control_in;
    R2_QP_(i, i) = beta_control_in;
  }
}

void VariationalBasedOptimalController::setWorldData() {
  direction_normal_flat_ground_ << 0, 0, 1;
  gravity_ << 0, 0, 9.81;
  direction_tangential_flat_ground_ << 0.7071, 0.7071, 0;
  mu_friction_ = 0.05;
}

void VariationalBasedOptimalController::setWorldData(double* direction_normal_flat_ground, double* gravity,
                                                     double* direction_tangential_flag_ground, double mu_friction) {
  direction_normal_flat_ground_ << direction_normal_flat_ground[0], direction_normal_flat_ground[1],
      direction_normal_flat_ground[2];
  gravity_ << gravity[0], gravity[1], gravity[2];
  direction_tangential_flat_ground_ << direction_tangential_flag_ground[0], direction_tangential_flag_ground[1],
      direction_tangential_flag_ground[2];
  mu_friction_ = mu_friction;
}

void VariationalBasedOptimalController::setFriction(double mu_in) { mu_friction_ = mu_in; }

void VariationalBasedOptimalController::setMass(double mass_in) { mass_ = mass_in; }

void VariationalBasedOptimalController::setInertia(double Ixx, double Iyy, double Izz) { Ig_ << Ixx, 0, 0, 0, Iyy, 0, 0, 0, Izz; }

void VariationalBasedOptimalController::setRobotLimits(double min_force, double max_force) {
  minNormalForces_feet_ << min_force, min_force, min_force, min_force;
  maxNormalForces_feet_ << max_force, max_force, max_force, max_force;
}

/* ------------ Utilities -------------- */

bool VariationalBasedOptimalController::getQPFinished() { return QPFinished_; }

void VariationalBasedOptimalController::copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols) {
  int count = 0;

  // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      target[count] = source(i, j);
      count++;
    }
  }
}

void VariationalBasedOptimalController::copy_Eigen_to_double(double* target, Eigen::VectorXd& source, int length) {
  for (int i = 0; i < length; i++) {
    target[i] = source(i);
  }
}

void VariationalBasedOptimalController::copy_Array_to_Eigen(Eigen::VectorXd& target, double* source, int len, int startIndex) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i + startIndex];
  }
}

void VariationalBasedOptimalController::copy_Array_to_Eigen(Eigen::MatrixXd& target, double* source, int len, int startIndex) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i + startIndex];
  }
}

void VariationalBasedOptimalController::copy_real_t_to_Eigen(Eigen::VectorXd& target, real_t* source, int len) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i];
  }
}

void VariationalBasedOptimalController::matrixLogRot(const Eigen::MatrixXd& R, Eigen::VectorXd& omega) {
  // theta = acos( (Trace(R) - 1)/2 )
  double theta;
  double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
  if (tmp >= 1.) {
    theta = 0;
  } else if (tmp <= -1.) {
    theta = M_PI;
  } else {
    theta = acos(tmp);
  }

  // Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
  // crossExtract(omegaHat,omega);
  omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  if (theta > 10e-5) {
    omega *= theta / (2 * sin(theta));
  } else {
    omega /= 2;
  }
}

void VariationalBasedOptimalController::crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega) {
  R.setZero();
  R(0, 1) = -omega(2);
  R(0, 2) = omega(1);
  R(1, 0) = omega(2);
  R(1, 2) = -omega(0);
  R(2, 0) = -omega(1);
  R(2, 1) = omega(0);
}

void VariationalBasedOptimalController::inverseCrossMatrix(const Eigen::MatrixXd& R, Eigen::VectorXd& omega) {
  omega(0) = R(2, 1);
  omega(1) = R(0, 2);
  omega(2) = R(1, 0);
}

void VariationalBasedOptimalController::matrixExpOmegaCross(const Eigen::VectorXd& omega, Eigen::MatrixXd& R) {
  double theta = omega.norm();
  R.setIdentity();

  if (theta > 1e-9) {
    omegaHat_.setZero();
    crossMatrix(omegaHat_, omega / theta);
    // R = I + omegaHat sin(theta) + omegaHat^2 (1-cos(theta))
    R += omegaHat_ * sin(theta) + omegaHat_ * omegaHat_ * (1 - cos(theta));
  }
}

void VariationalBasedOptimalController::quaternion_to_rotationMatrix(Eigen::MatrixXd& R, Eigen::VectorXd& quat) {
  // wikipedia
  // reference link: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  R(0, 0) = 1 - 2 * quat(2) * quat(2) - 2 * quat(3) * quat(3);
  R(0, 1) = 2 * quat(1) * quat(2) - 2 * quat(0) * quat(3);
  R(0, 2) = 2 * quat(1) * quat(3) + 2 * quat(0) * quat(2);
  R(1, 0) = 2 * quat(1) * quat(2) + 2 * quat(0) * quat(3);
  R(1, 1) = 1 - 2 * quat(1) * quat(1) - 2 * quat(3) * quat(3);
  R(1, 2) = 2 * quat(2) * quat(3) - 2 * quat(1) * quat(0);
  R(2, 0) = 2 * quat(1) * quat(3) - 2 * quat(2) * quat(0);
  R(2, 1) = 2 * quat(2) * quat(3) + 2 * quat(1) * quat(0);
  R(2, 2) = 1 - 2 * quat(1) * quat(1) - 2 * quat(2) * quat(2);
}

void VariationalBasedOptimalController::rpyToR(Eigen::MatrixXd& R, double* rpy_in) {
  Eigen::Matrix3d Rz, Ry, Rx;

  Rz.setIdentity();
  Ry.setIdentity();
  Rx.setIdentity();

  Rz(0, 0) = cos(rpy_in[2]);
  Rz(0, 1) = -sin(rpy_in[2]);
  Rz(1, 0) = sin(rpy_in[2]);
  Rz(1, 1) = cos(rpy_in[2]);

  Ry(0, 0) = cos(rpy_in[1]);
  Ry(0, 2) = sin(rpy_in[1]);
  Ry(2, 0) = -sin(rpy_in[1]);
  Ry(2, 2) = cos(rpy_in[1]);

  Rx(1, 1) = cos(rpy_in[0]);
  Rx(1, 2) = -sin(rpy_in[0]);
  Rx(2, 1) = sin(rpy_in[0]);
  Rx(2, 2) = cos(rpy_in[0]);

  R = Rz * Ry * Rx;
}