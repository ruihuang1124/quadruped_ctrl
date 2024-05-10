//
// Created by ray on 12/12/23.
//
#include "VBOCWrapper.h"

#include <cstdlib>
#include <iostream>

#include "VariationalBasedOptimalController.h"

#ifdef __cplusplus
extern "C" {
#endif

static VariationalBasedOptimalController* VBOptimalController = NULL;
double xOpt_force_[12];

void vboc_init() {
  if (VBOptimalController == NULL) {
    VBOptimalController = new VariationalBasedOptimalController();
    std::cout << "debug initialize variational based optimal controller\n";
  }
}

void vboc_solveQP_nonThreaded() { VBOptimalController->solveQP_nonThreaded(xOpt_force_); }

double vbOptimalControl_get_fOpt_matlab(int index) { return xOpt_force_[index - 1]; }

void vboc_print() { printf("variational Based Optimal Controller here\n\n"); }

void get_vboc_solution(double* xOpt) {
  for (int i = 0; i < 12; ++i) {
    xOpt[i] = xOpt_force_[i];
  }
}

// Set
void set_vboc_world_data(double* direction_normal_flat_ground, double* gravity, double* direction_tangetial_flag_ground,
                         double mu_friction) {
  VBOptimalController->setWorldData(direction_normal_flat_ground, gravity, direction_tangetial_flag_ground, mu_friction);
}

void set_all_vboc_solver_parameters(double* x_weights_in, double* xdot_weights_in, double* R_weights_in, double* omega_weights_in,
                                    double alpha_control_in, double beta_control_in, double mu_in, double mass_in, double Ixx,
                                    double Iyy, double Izz, double min_force, double max_force) {
  VBOptimalController->setVBOCSolverConfig(x_weights_in, xdot_weights_in, R_weights_in, omega_weights_in, alpha_control_in,
                                           beta_control_in, mu_in, mass_in, Ixx, Iyy, Izz, min_force, max_force);
}

// Update
void vboc_update_desired_trajectory_data(double* rpy_des_in, double* p_des_in, double* omega_des_in,
                                         double* v_des_in)  //, double* vdot_des_in)
{
  VBOptimalController->updateDesiredTrajectoryData(rpy_des_in, p_des_in, omega_des_in, v_des_in);  //, vdot_des_in );
}

void vboc_update_contact_data(double* contact_state_in, double* min_forces_in, double* max_forces_in, double threshold,
                              int stance_legs_in) {
  VBOptimalController->updateContactData(contact_state_in, min_forces_in, max_forces_in, threshold, stance_legs_in);
}

void vboc_update_reference_GRF(double* f_ref_in) { VBOptimalController->updateReference_GRF(f_ref_in); }

void vboc_update_problem_data(double* state_x_feedback_in, double* p_feet_in, double* p_feet_desired_in, double* rpy_des_in,
                              double* rpy_act_in) {
  VBOptimalController->updateProblemData(state_x_feedback_in, p_feet_in, p_feet_desired_in, rpy_des_in, rpy_act_in);
}

#ifdef __cplusplus
}
#endif
