//
// Created by ray on 12/13/23.
//

#ifndef ARCDOG_SOFTWARE_VBOCLOCOMOTION_H
#define ARCDOG_SOFTWARE_VBOCLOCOMOTION_H

#include "../ConvexMPCLocomotion.h"
#include "VariationalBasedOptimalController.h"
#include "VBOCWrapper.h"

class VBOCLocomotion : public ConvexMPCLocomotion{
public:
    VBOCLocomotion(float _dt, int _iterations_between_mpc);

    template<typename T>
    void runVBOC(Quadruped<T> &_quadruped, LegController<T> &_legController, StateEstimatorContainer<float> &_stateEstimator,
             DesiredStateCommand<T> &_desiredStateCommand, std::vector<double> gamepadCommand, int gaitType, int robotMode = 0);
private:

    void updateVBOCMPCIfNeeded(StateEstimatorContainer<float> &_stateEstimator, bool omniMode);
    void setVBOCSolverParameters();
    void solveVBMPC(StateEstimatorContainer<float> &_stateEstimator);
    bool mix_control_command_;
//  robot_control_command_mpc_lcmt mix_control_command_lcm_;

    double x_weight_in_[3], xdot_weights_in_[3], R_weights_in_[3], omega_weights_in_[3], alpha_control_in_, beta_control_in_,
            mu_in_, mass_in_, Ixx_, Iyy_, Izz_, min_force_, max_force_;
};

#endif  // ARCDOG_SOFTWARE_VBOCLOCOMOTION_H
