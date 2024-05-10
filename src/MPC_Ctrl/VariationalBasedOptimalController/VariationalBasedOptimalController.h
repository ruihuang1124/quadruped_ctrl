//
// Created by ray on 12/12/23.
//

#ifndef ARCDOG_SOFTWARE_VARIATIONALBASEDOPTIMALCONTROLLER_H
#define ARCDOG_SOFTWARE_VARIATIONALBASEDOPTIMALCONTROLLER_H

#include <eigen3/Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
#include <qpOASES/include/qpOASES.hpp>

//#include "sim_command_t.hpp"
//#include "two_contact_stand_data_t.hpp"

static const int VBOC_NUM_VARIABLES_QP = 12;
static const int VBOC_NUM_CONSTRAINTS_QP = 20;
static const int VBOC_NUM_CONTACT_POINTS = 4;
static const int VBOC_NUM_VARIABLES_PER_FOOT = 3;
static const int VBOC_NUM_CONSTRAINTS_PER_FOOT = 5;

// static const double PI_CONST = 3.1415;
static const double VBOC_NEGATIVE_NUMBER = -1000000.0;
static const double VBOC_POSITIVE_NUMBER = 1000000.0;

using namespace Eigen;
using namespace qpOASES;

class VariationalBasedOptimalController {
 public:
  VariationalBasedOptimalController();
  ~VariationalBasedOptimalController(){};

  // use new kinematics measurements to update QP
  void updateProblemData(double* state_x_feedback_in, double* p_feet_in, double* p_feet_desired_in, double* rpy_des_in,
                         double* rpy_act_in);

  void updateContactData(double* contact_state_in, double* min_forces_in, double* max_forces_in, double threshold,
                         int stance_legs_in);

  // update desired COM and orientation setpoints
  void updateDesiredTrajectoryData(double* rpy_des_in, double* p_des_in, double* omega_des_in, double* v_des_in);
  void updateReference_GRF(double* f_ref_in);

  // calculate the QP, return solution
  void solveQP(double* xOpt);
  void solveQP_nonThreaded(double* xOpt);

  // configure gains, QP weights, force limits, world parameters
  void setVBOCSolverConfig(double* x_weights_in, double* xdot_weights_in, double* R_weights_in, double* omega_weights_in,
                           double alpha_control_in, double beta_control_in, double mu_in, double mass_in, double Ixx, double Iyy,
                           double Izz, double min_force, double max_force);
  void setWorldData(double* direction_normal_flat_ground, double* gravity, double* direction_tangetial_flag_ground,
                    double mu_friction);

  Eigen::VectorXd xOpt_combined;

 private:
  void setRobotLimits(double min_force, double max_force);
  void setWorldData();
  void setLQRWeights(double* x_weights_in, double* xdot_weights_in, double* R_weights_in, double* omega_weights_in,
                     double alpha_control_in, double beta_control_in);  // new
  void setFriction(double mu_in);
  void setMass(double mass_in);
  void setInertia(double Ixx, double Iyy, double Izz);

  // Misc
  void setBaseSupportFlag(double sflag);

//  lcm::LCM* lcm_;
//  two_contact_stand_data_t two_contact_stand_data_, two_contact_stand_data_publish_;
  //  sim_command_t command;

  /* Fixed-Size qpOASES data */
  QProblem QProblemObj_qpOASES_;

  int_t nWSR_qpOASES_ = 100;
  int_t nWSR_fixed_ = 100;

  real_t cpu_time_;
  real_t cpu_time_fixed_;

  int_t qp_exit_flag_;

  int nWSR_initial_;
  double cpu_time_initial_;
  //
  //  double xOpt_local[12];  // delete?
  double qp_not_init_;

  Bounds guessedBounds_;
  Constraints guessedConstraints_;

  /* QP variables for HJB optimization */
  real_t H_qpOASES_[VBOC_NUM_VARIABLES_QP * VBOC_NUM_VARIABLES_QP];
  real_t A_qpOASES_[VBOC_NUM_CONSTRAINTS_QP * VBOC_NUM_VARIABLES_QP];
  real_t g_qpOASES_[VBOC_NUM_VARIABLES_QP];
  real_t lb_qpOASES_[VBOC_NUM_VARIABLES_QP];
  real_t ub_qpOASES_[VBOC_NUM_VARIABLES_QP];
  real_t lbA_qpOASES_[VBOC_NUM_CONSTRAINTS_QP];
  real_t ubA_qpOASES_[VBOC_NUM_CONSTRAINTS_QP];
  real_t xOpt_qpOASES_[VBOC_NUM_VARIABLES_QP];
  real_t yOpt_qpOASES_[VBOC_NUM_VARIABLES_QP + VBOC_NUM_CONSTRAINTS_QP];

  real_t xOpt_initialGuess_[VBOC_NUM_VARIABLES_QP];

  /* Eigen Variables that Match qpOASES variables */
  Eigen::MatrixXd H_eigen_;
  Eigen::MatrixXd A_eigen_;
  Eigen::MatrixXd g_eigen_;
  Eigen::VectorXd xOpt_eigen_;
  Eigen::VectorXd yOpt_eigen_;
  Eigen::VectorXd xOptPrev_;
  Eigen::VectorXd yOptPrev_;

  /* Robot control variables used to construct QP matrices, see (5) and (6) of [R1] */
  Eigen::MatrixXd C_control_;
  Eigen::VectorXd C_times_f_opt_;
  Eigen::MatrixXd R1_QP_;
  Eigen::MatrixXd R2_QP_;
  Eigen::MatrixXd B_QP_;

  /* NEW - Robot control variables used in LQR*/
  Eigen::MatrixXd A_LQR_;
  Eigen::MatrixXd B_LQR_;
  Eigen::MatrixXd P_LQR_;
  Eigen::VectorXd f_ref_world_;
  Eigen::MatrixXd Q1_LQR_;
  Eigen::MatrixXd R_LQR_;
  Eigen::VectorXd s_LQR_;
  Eigen::MatrixXd H_LQR_;
  Eigen::VectorXd f_unc_;
  double cost_to_go_;

  /* Model and World parameters and force limits */
  double mass_;
  double inertia_;
  Eigen::MatrixXd Ig_;  // where does this get set? - beside constructor?

  double mu_friction_;
  Eigen::VectorXd gravity_;

  Eigen::VectorXd minNormalForces_feet_;
  Eigen::VectorXd maxNormalForces_feet_;

  Eigen::VectorXd direction_normal_flat_ground_;
  Eigen::VectorXd direction_tangential_flat_ground_;

  /* Foot Contact Information, 1 is on the ground,  */
  Eigen::VectorXd contact_state_;
  double threshold_;
  int contact_legs_;
  int leg_cnt_;
  int stance_legs_;

  /* Actual Kinematics*/
  Eigen::VectorXd x_COM_world_;
  Eigen::VectorXd xdot_COM_world_;
  Eigen::VectorXd omega_b_world_;
  Eigen::VectorXd omega_b_body_;  // new
  Eigen::VectorXd quat_b_world_;
  Eigen::MatrixXd R_b_world_;
  Eigen::MatrixXd p_feet_;

  /* Desired Kinematics */
  Eigen::VectorXd x_COM_world_desired_;
  Eigen::VectorXd xdot_COM_world_desired_;
  Eigen::VectorXd xddot_COM_world_desired_;
  Eigen::VectorXd omega_b_world_desired_;
  Eigen::VectorXd omega_b_body_desired_;  // new
  Eigen::VectorXd omega_dot_b_world_desired_;
  Eigen::MatrixXd R_b_world_desired_;  // orientation of the body frame{B} with respect to the inertial frame {o}.
  Eigen::MatrixXd p_feet_desired_;     // new

  // Error coordinates - removed PD control error variables
  Eigen::VectorXd vbd_command_eigen_;
  Eigen::VectorXd orientation_error_;
  Eigen::VectorXd error_x_lin_;      // new
  Eigen::VectorXd error_dx_lin_;     // new
  Eigen::VectorXd error_R_lin_;      // new
  Eigen::VectorXd error_omega_lin_;  // new

  /* Temporary, Internal Matrices */
  Eigen::MatrixXd omegaHat_;
  Eigen::MatrixXd tempSkewMatrix3_;
  Eigen::VectorXd tempVector3_;
  Eigen::MatrixXd tempSkewMatrix4_;

  /* Interal QP management data */
  bool QPFinished_;

  /* Interface Functions */
  bool getQPFinished();

  void update_A_control();
  void calc_linear_error();
  void update_A_LQR();
  void update_B_LQR();
  void update_P_LQR();

  void calc_H_qpOASES();
  void calc_A_qpOASES();
  void calc_g_qpOASES();
  void calc_lb_ub_qpOASES();
  void calc_lbA_ubA_qpOASES();

  void calc_constraint_check();

  /* Utility Functions */
  void copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols);
  void copy_Eigen_to_double(double* target, Eigen::VectorXd& source, int length);
  void copy_Array_to_Eigen(Eigen::VectorXd& target, double* source, int len, int startIndex);
  void copy_Array_to_Eigen(Eigen::MatrixXd& target, double* source, int len, int startIndex);
  void copy_real_t_to_Eigen(Eigen::VectorXd& target, real_t* source, int len);

  void matrixExpOmegaCross(const Eigen::VectorXd& omega, Eigen::MatrixXd& R);
  void matrixLogRot(const Eigen::MatrixXd& R, Eigen::VectorXd& omega);
  void crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega);
  void inverseCrossMatrix(const Eigen::MatrixXd& R, Eigen::VectorXd& omega);     // New function, need to test
  void quaternion_to_rotationMatrix(Eigen::MatrixXd& R, Eigen::VectorXd& quat);  // quat = {w, x, y, z};

  void rpyToR(Eigen::MatrixXd& R, double* rpy_in);
};

#endif  // ARCDOG_SOFTWARE_VARIATIONALBASEDOPTIMALCONTROLLER_H
