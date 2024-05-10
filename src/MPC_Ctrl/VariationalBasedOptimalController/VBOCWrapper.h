//
// Created by ray on 12/12/23.
//

#ifndef ARCDOG_SOFTWARE_VBOCWRAPPER_H
#define ARCDOG_SOFTWARE_VBOCWRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * init, solve & other from update and set
 */
void vboc_init();
void vboc_solveQP_nonThreaded();
void get_vboc_solution(double* xOpt);
void vboc_print();
/**
// Set
*/
void set_vboc_world_data(double* direction_normal_flat_ground, double* gravity, double* direction_tangetial_flag_ground,
                         double mu_friction);
void set_all_vboc_solver_parameters(double* x_weights_in, double* xdot_weights_in, double* R_weights_in, double* omega_weights_in,
                                    double alpha_control_in, double beta_control_in, double mu_in, double mass_in, double Ixx,
                                    double Iyy, double Izz, double min_force, double max_force);

/**
// Update
 */

void vboc_update_desired_trajectory_data(double* rpy_des_in, double* p_des_in, double* omega_des_in,
                                         double* v_des_in);  //, double *vdot_des_in);
void vboc_update_contact_data(double* contact_state_in, double* min_forces_in, double* max_forces_in, double threshold,
                              int stance_legs_in);
void vboc_update_reference_GRF(double* f_ref_in);
void vboc_update_problem_data(double* state_x_feedback_in, double* p_feet_in, double* p_feet_desired_in, double* rpy_des_in,
                              double* rpy_act_in);

#ifdef __cplusplus
}
#endif

#endif  // ARCDOG_SOFTWARE_VBOCWRAPPER_H
