//
// Created by ray on 12/13/23.
//
#include "VBOCLocomotion.h"


VBOCLocomotion::VBOCLocomotion(float _dt, int _iterations_between_mpc) : ConvexMPCLocomotion(_dt,
                                                                                             _iterations_between_mpc) {
    vboc_init();
    setVBOCSolverParameters();
}

void VBOCLocomotion::setVBOCSolverParameters() {
    x_weight_in_[0] = 20;
    x_weight_in_[1] = 20;
    x_weight_in_[2] = 1000;
    xdot_weights_in_[0] = 1;
    xdot_weights_in_[1] = 1;
    xdot_weights_in_[2] = 10;
    R_weights_in_[0] = 1100;
    R_weights_in_[1] = 1600;
    R_weights_in_[2] = 500;
    omega_weights_in_[0] = 10;
    omega_weights_in_[1] = 5;
    omega_weights_in_[2] = 1;
    alpha_control_in_ = 1.25;
    beta_control_in_ = 2.25;
    mu_in_ = 1.0;
    min_force_ = 10.0;
    max_force_ = 160.0;

    mass_in_ = 3.3;
    Ixx_ = 0.011253;
    Iyy_ = 0.036203;
    Izz_ = 0.042673;
    set_all_vboc_solver_parameters(x_weight_in_, xdot_weights_in_, R_weights_in_, omega_weights_in_, alpha_control_in_,
                                   beta_control_in_, mu_in_, mass_in_, Ixx_, Iyy_, Izz_, min_force_, max_force_);
}

template<>
void VBOCLocomotion::runVBOC(Quadruped<float> &_quadruped, LegController<float> &_legController,
                             StateEstimatorContainer<float> &_stateEstimator,
                             DesiredStateCommand<float> & /*_desiredStateCommand*/, std::vector<double> gamepadCommand,
                             int gaitType, int robotMode) {
    bool omniMode = false;
    // Command Setup
    _SetupCommand(_stateEstimator, gamepadCommand);

    gaitNumber = gaitType;  // data.userParameters->cmpc_gait; 步态默认为trot

    if (gaitNumber >= 20) {
        gaitNumber -= 20;
        omniMode = true;
    }

    auto &seResult = _stateEstimator.getResult();  //状态估计器

    // Check if transition to standing 检查是否过渡到站立
    if (((gaitNumber == 4) && current_gait != 4) || firstRun) {
        stand_traj[0] = seResult.position[0];
        stand_traj[1] = seResult.position[1];
        stand_traj[2] = 0.21;
        stand_traj[3] = 0;
        stand_traj[4] = 0;
        stand_traj[5] = seResult.rpy[2];
        world_position_desired[0] = stand_traj[0];
        world_position_desired[1] = stand_traj[1];
    }

    // pick gait
    Gait *gait = &trotting;
    if (robotMode == 0) {
        if (gaitNumber == 1)
            gait = &bounding;
        else if (gaitNumber == 2)
            gait = &pronking;
            // else if(gaitNumber == 3)
            //   gait = &random;
        else if (gaitNumber == 4)
            gait = &standing;
        else if (gaitNumber == 5)
            gait = &trotRunning;
            // else if(gaitNumber == 6)
            //   gait = &random2;
        else if (gaitNumber == 7)
            gait = &galloping;
        else if (gaitNumber == 8)
            gait = &pacing;
        else if (gaitNumber == 9)
            gait = &trotting;
        else if (gaitNumber == 10)
            gait = &walking;
        else if (gaitNumber == 11)
            gait = &walking2;
    } else if (robotMode == 1) {
        int h = 10;
        double vBody = sqrt(_x_vel_des * _x_vel_des) + (_y_vel_des * _y_vel_des);
        gait = &aio;
        gaitNumber = 9;  // Trotting
        if (gait->getCurrentGaitPhase() == 0) {
            if (vBody < 0.002) {
                if (abs(_yaw_turn_rate) < 0.01) {
                    gaitNumber = 4;  // Standing
                    if (gait->getGaitHorizon() != h) {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h, Vec4<int>(0, 0, 0, 0), Vec4<int>(h, h, h, h), "Standing");
                } else {
                    h = 10;
                    if (gait->getGaitHorizon() != h) {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                                       Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");
                }
            } else {
                if (vBody <= 0.2) {
                    h = 16;
                    if (gait->getGaitHorizon() != h) {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h,
                                       Vec4<int>(0, 1 * h / 2, 1 * h / 4, 3 * h / 4),
                                       Vec4<int>(3 * h / 4, 3 * h / 4, 3 * h / 4, 3 * h / 4), "Walking");
                } else if (vBody > 0.2 && vBody <= 0.4) {
                    h = 16;
                    if (gait->getGaitHorizon() != h) {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h,
                                       Vec4<int>(0, 1 * h / 2, h * ((5.0 / 4.0) * vBody),
                                                 h * ((5.0 / 4.0) * vBody + (1.0 / 2.0))),
                                       Vec4<int>(h * ((-5.0 / 4.0) * vBody + 1.0), h * ((-5.0 / 4.0) * vBody + 1.0),
                                                 h * ((-5.0 / 4.0) * vBody + 1.0), h * ((-5.0 / 4.0) * vBody + 1.0)),
                                       "Walking2trotting");
                } else if (vBody > 0.4 && vBody <= 1.4) {
                    h = 14;
                    if (gait->getGaitHorizon() != h) {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                                       Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");
                } else {
                    // h = 10;
                    h = -20.0 * vBody + 42.0;
                    if (h < 10) h = 10;
                    if (gait->getGaitHorizon() != h) {
                        iterationCounter = 0;
                    }
                    gait->setGaitParam(h, Vec4<int>(0, h / 2, h / 2, 0),
                                       Vec4<int>(h / 2, h / 2, h / 2, h / 2), "trotting");

                    // std::cout << vBody << " " << h << " " << h / 2 << std::endl;
                }
            }
        }
        horizonLength = h;
    } else {
        std::cout << "err robot mode!!!" << std::endl;
    }

    current_gait = gaitNumber;
    gait->setIterations(iterationsBetweenMPC, iterationCounter);  //步态周期计算

    // integrate position setpoint
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,
                            0);  //身体坐标系下的期望线速度
    Vec3<float> v_des_world = omniMode
                              ? v_des_robot
                              : seResult.rBody.transpose() *
                                v_des_robot;  //世界坐标系下的期望线速度
    Vec3<float> v_robot = seResult.vWorld;  //世界坐标系下的机器人实际速度

    // Integral-esque pitche and roll compensation
    // 积分达到补偿值*******************************
    // 为了保持在运动过程中身体与地面平行
    if (fabs(v_robot[0]) > .2)  // avoid dividing by zero
    {
        rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
    }
    if (fabs(v_robot[1]) > 0.1) {
        rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
    }

    //初始角度限幅
    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);  //-0.25~0.25
    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
    rpy_comp[1] = v_robot[0] * rpy_int[1];  // compensation 补偿值
    rpy_comp[0] = v_robot[1] * rpy_int[0];  // turn off for pronking

    //得到世界坐标系下的足端位置
    //机身坐标+机身旋转矩阵^T*（侧摆关节在机身下坐标+足底在侧摆关节下坐标）
    for (int i = 0; i < 4; i++) {
        pFoot[i] = seResult.position +
                   seResult.rBody.transpose() *
                   (_quadruped.getHipLocation(i) + _legController.datas[i].p);
    }

    Vec3<float> error;
    if (gait != &standing) {  //非站立下的期望位置，通过累加期望速度完成
        world_position_desired +=
                dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
    }

    // some first time initialization
    if (firstRun) {
        world_position_desired[0] = seResult.position[0];
        world_position_desired[1] = seResult.position[1];
        world_position_desired[2] = seResult.rpy[2];

        for (int i = 0; i < 4; i++)  //足底摆动轨迹
        {
            footSwingTrajectories[i].setHeight(0.06);
            footSwingTrajectories[i].setInitialPosition(pFoot[i]);  // set p0
            footSwingTrajectories[i].setFinalPosition(pFoot[i]);    // set pf
            pFoot_des[i] = seResult.position +
                       seResult.rBody.transpose() *
                       (_quadruped.getHipLocation(i) + _legController.datas[i].p);
        }
        firstRun = false;
    }

    // foot placement
    for (int l = 0; l < 4; l++) {
        swingTimes[l] = gait->getCurrentSwingTime(
                dtMPC, l);  // return dtMPC * _stance  0.026 * 5 = 0.13
        // dtMPC的值变为了0.026，外部给修改的赋值
    }

    float side_sign[4] = {-1, 1, -1, 1};
    float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
    // float interleave_gain = -0.13;
    float interleave_gain = -0.2;
    // float v_abs = std::fabs(seResult.vBody[0]);
    float v_abs = std::fabs(v_des_robot[0]);
    for (int i = 0; i < 4; i++) {
        if (firstSwing[i]) {
            swingTimeRemaining[i] = swingTimes[i];
        } else {
            swingTimeRemaining[i] -= dt;
        }

        footSwingTrajectories[i].setHeight(0.06);
        Vec3<float> offset(0, side_sign[i] * .065, 0);

        Vec3<float> pRobotFrame = (_quadruped.getHipLocation(i) +
                                   offset);  //得到身体坐标系下的hip关节坐标

        pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
        float stance_time =
                gait->getCurrentStanceTime(dtMPC, i);  // stance_time = 0.13

        Vec3<float> pYawCorrected =
                coordinateRotation(CoordinateAxis::Z,
                                   -_yaw_turn_rate * stance_time / 2) *
                pRobotFrame;  //机身旋转yaw后，得到在机身坐标系下的hip坐标

        Vec3<float> des_vel;
        des_vel[0] = _x_vel_des;
        des_vel[1] = _y_vel_des;
        des_vel[2] = 0.0;

        //世界坐标系下hip坐标 以剩余摆动时间内匀速运动来估计
        Vec3<float> Pf = seResult.position +
                         seResult.rBody.transpose() *
                         (pYawCorrected + des_vel * swingTimeRemaining[i]);
        // float p_rel_max = 0.35f;
        float p_rel_max = 0.3f;

        // Using the estimated velocity is correct
        // Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
        float pfx_rel = seResult.vWorld[0] * (.5 + 0.0) *
                        stance_time +  //_parameters->cmpc_bonus_swing = 0.0
                        .03f * (seResult.vWorld[0] - v_des_world[0]) +
                        (0.5f * sqrt(seResult.position[2] / 9.81f)) *
                        (seResult.vWorld[1] * _yaw_turn_rate);

        float pfy_rel = seResult.vWorld[1] * .5 * stance_time * 1.0 + //dtMPC +
                        .03f * (seResult.vWorld[1] - v_des_world[1]) +
                        (0.5f * sqrt(seResult.position[2] / 9.81f)) *
                        (-seResult.vWorld[0] * _yaw_turn_rate);
        if (i == 1) {
            // std::cout << "pf0 = " << (seResult.rBody*(Pf- seResult.position)).transpose() << " " << std::endl;
            // std::cout << pfx_rel << " " << pfy_rel << std::endl;
            // std::cout << 0.5f * sqrt(seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate) << " "
            //           << (0.5f * sqrt(seResult.position[2] / 9.81f)) * (-seResult.vWorld[0] * _yaw_turn_rate) << std::endl;
            // std::cout << (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[0] * _yaw_turn_rate) << std::endl;
            // std::cout << _yaw_turn_rate << std::endl;
        }
        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel;
        // Pf[2] = -0.003;
        Pf[2] = 0.0;
        footSwingTrajectories[i].setFinalPosition(Pf);  //最终得到足底的位置，并作为轨迹终点 世界坐标系下的落足点
    }
    // std::cout << std::endl;

    // calc gait
    iterationCounter++;

    // load LCM leg swing gains
    Kp << 700, 0, 0, 0, 700, 0, 0, 0, 200;
    Kp_stance = 0.0 * Kp;

    Kd << 10, 0, 0, 0, 10, 0, 0, 0, 10;
    Kd_stance = 1.0 * Kd;
    // gait
    Vec4<float> contactStates = gait->getContactState();
    contact_state = gait->getContactState();
    Vec4<float> swingStates = gait->getSwingState();
    int *mpcTable = gait->getMpcTable();
    double use_vboc = 2.0;
    if (use_vboc > 1){
        updateVBOCMPCIfNeeded(_stateEstimator,omniMode);
    } else{
        updateMPCIfNeeded(mpcTable, _stateEstimator, omniMode);
    }

    //  StateEstimator* se = hw_i->state_estimator;
    Vec4<float> se_contactState(0, 0, 0, 0);

    bool use_wbc = false;

    for (int foot = 0; foot < 4; foot++) {
        float contactState = contactStates[foot];
        float swingState = swingStates[foot];
        if (swingState > 0)  // foot is in swing
        {
            if (firstSwing[foot]) {
                firstSwing[foot] = false;
                footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
            }

            footSwingTrajectories[foot].computeSwingTrajectoryBezier(
                    swingState, swingTimes[foot]);

            Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
            Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();

            Vec3<float> pDesLeg =
                    seResult.rBody *
                    (pDesFootWorld -
                     seResult.position)  //侧摆关节坐标系下的足端坐标
                    //(此处先改为身体坐标系下的足端坐标)
                    - _quadruped.getHipLocation(foot);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            // Update for WBC
            pFoot_des[foot] = pDesFootWorld;
            vFoot_des[foot] = vDesFootWorld;
            aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

            if (!use_wbc) {
                // Update leg control command regardless of the usage of WBIC
                _legController.commands[foot].pDes = pDesLeg;
                _legController.commands[foot].vDes = vDesLeg;
                if (foot == 1 || foot == 3) {
                    _legController.commands[foot].kpCartesian = Kp;
                    _legController.commands[foot].kdCartesian = Kd;
                } else {
                    _legController.commands[foot].kpCartesian = 1 * Kp;
                    _legController.commands[foot].kdCartesian = 1 * Kd;
                }
            }
        } else  // foot is in stance
        {
            firstSwing[foot] = true;

            Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
            Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
            Vec3<float> pDesLeg =
                    seResult.rBody * (pDesFootWorld - seResult.position) -
                    _quadruped.getHipLocation(foot);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            if (!use_wbc) {
                _legController.commands[foot].pDes = pDesLeg;
                _legController.commands[foot].vDes = vDesLeg;

                if (foot == 1 || foot == 3) {
                    _legController.commands[foot].kdCartesian = Kd_stance;
                } else {
                    _legController.commands[foot].kdCartesian = 1 * Kd_stance;
                }

                _legController.commands[foot].forceFeedForward = f_ff[foot];
                _legController.commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

            } else {  // Stance foot damping
                _legController.commands[foot].pDes = pDesLeg;
                _legController.commands[foot].vDes = vDesLeg;
                _legController.commands[foot].kpCartesian = 0. * Kp_stance;
                _legController.commands[foot].kdCartesian = Kd_stance;
            }
            se_contactState[foot] = contactState;

            // Update for WBC
            // Fr_des[foot] = -f_ff[foot];
        }
    }
    // se->set_contact_state(se_contactState); todo removed
    _stateEstimator.setContactPhase(se_contactState);

    // Update For WBC
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = _body_height;

    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0.;

    aBody_des.setZero();

    pBody_RPY_des[0] = 0.;
    pBody_RPY_des[1] = 0.;
    pBody_RPY_des[2] = _yaw_des;

    vBody_Ori_des[0] = 0.;
    vBody_Ori_des[1] = 0.;
    vBody_Ori_des[2] = _yaw_turn_rate;

    // contact_state = gait->getContactState();
    contact_state = gait->getContactState();
    // END of WBC Update
}

void VBOCLocomotion::updateVBOCMPCIfNeeded(StateEstimatorContainer<float> &_stateEstimator, bool omniMode) {
    // iterationsBetweenMPC = 30;
    if ((iterationCounter % iterationsBetweenMPC) == 0) {
        auto seResult = _stateEstimator.getResult();
        float* p = seResult.position.data();

        Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
        Vec3<float> v_des_world =
                omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
        // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

        // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
        // x_comp_integral);

        if (current_gait == 4) {
            float trajInitial[12] = {
                    _roll_des,
                    _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
                    (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
                    (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
                    (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
                    (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0};

            for (int i = 0; i < horizonLength; i++)
                for (int j = 0; j < 12; j++) trajAll[12 * i + j] = trajInitial[j];
        }

        else {
            const float max_pos_error = .1;
            float xStart = world_position_desired[0];
            float yStart = world_position_desired[1];

            if (xStart - p[0] > max_pos_error) xStart = p[0] + 0.1;
            if (p[0] - xStart > max_pos_error) xStart = p[0] - 0.1;

            if (yStart - p[1] > max_pos_error) yStart = p[1] + 0.1;
            if (p[1] - yStart > max_pos_error) yStart = p[1] - 0.1;

            world_position_desired[0] = xStart;
            world_position_desired[1] = yStart;

            float trajInitial[12] = {(float)rpy_comp[0],  // 0
                                     (float)rpy_comp[1],  // 1
                                     _yaw_des_true,            // 2
                    // yawStart,    // 2
                                     xStart,               // 3
                                     yStart,               // 4
                                     (float)_body_height,  // 5
                                     0,                    // 6
                                     0,                    // 7
                                     _yaw_turn_rate,       // 8
                                     v_des_world[0],       // 9
                                     v_des_world[1],       // 10
                                     0};                   // 11

            for (int i = 0; i < horizonLength; i++) {
                for (int j = 0; j < 12; j++) trajAll[12 * i + j] = trajInitial[j];

                if (i == 0)  // start at current position  TODO consider not doing this
                {
                    // trajAll[2] = seResult.rpy[2];
                    trajAll[2] = _yaw_des_true;
                } else {
                    trajAll[12 * i + 3] =
                            trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
                    trajAll[12 * i + 4] =
                            trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
                    trajAll[12 * i + 2] =
                            trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
                }
            }
        }

        double rpy_des_in[3], p_des_in[3], omega_des_in[3], v_des_in[3], contact_state_in[4], min_forces_in[4], max_forces_in[4],
                 state_x_feedback_in[13], rpy_act_in[3], p_feet_in[12], p_feet_desired_in[12];
        double threshold = 0.001;
        int stance_legs_in = 4;
        rpy_des_in[0] = _roll_des;
        rpy_des_in[1] = _pitch_des;
        rpy_des_in[2] = _yaw_des;
        p_des_in[0] = world_position_desired[0];
        p_des_in[1] = world_position_desired[1];
        p_des_in[2] = _body_height;

        Vec3<double> omega_des_world(0.0,0.0,0.0);

        // state_x_feedback_in = {qua_w, qua_x, qua_y, qua_z, x, y, z, roll_rate_world, pitch_rate_world, yaw_rate_world, vx, vy, vz}
        for (int i = 0; i < 3; ++i) {
            v_des_in[i] = v_des_world[i];
            omega_des_in[i] = omega_des_world[i];
            rpy_act_in[i] = seResult.rpy[i];
            state_x_feedback_in[4 + i] = seResult.position[i];
            state_x_feedback_in[7 + i] = seResult.omegaWorld[i];
            state_x_feedback_in[10 + i] = seResult.vWorld[i];
        }

        if (firstRunforVBOC_){
            for (int i = 0; i < 12; ++i) {
                f_ref_[i] = 0.0;
            }
            f_ref_[2] = 19.0;
            f_ref_[5] = 19.0;
            f_ref_[8] = 19.0;
            f_ref_[11] = 19.0;
//            firstRunforVBOC_ = false;
        }

        for (int i = 0; i < 4; ++i) {
            contact_state_in[i] = 1;
            min_forces_in[i] = 0.0;
            max_forces_in[i] = 160.0;
            state_x_feedback_in[i] = seResult.orientation[i];  // orientation = {w, x, y, z}
            for (int j = 0; j < 3; ++j) {
                p_feet_in[i * 3 + j] = pFoot[i][j];
                p_feet_desired_in[i * 3 + j] = pFoot_des[i][j];  // update pFoot_des correctly.
            }
        }
        // TODO the actual physical meaning of contact_state \in [0,1], also the threshold and stance_legs_in.

        vboc_update_desired_trajectory_data(rpy_des_in, p_des_in, omega_des_in, v_des_in);  //, double *vdot_des_in);
        vboc_update_contact_data(contact_state_in, min_forces_in, max_forces_in, threshold, stance_legs_in);
        vboc_update_reference_GRF(f_ref_);
        vboc_update_problem_data(state_x_feedback_in, p_feet_in, p_feet_desired_in, rpy_des_in, rpy_act_in);
        // Then invoke solver to solve the optimal control problem.
        solveVBMPC(_stateEstimator);
        // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
    }
}

void VBOCLocomotion::solveVBMPC(StateEstimatorContainer<float> &_stateEstimator) {
    auto seResult = _stateEstimator.getResult();
    vboc_solveQP_nonThreaded();
    double final_combined_force_robot_to_earth_robot_frame[12];
    get_vboc_solution(final_combined_force_robot_to_earth_robot_frame);

    for (int leg = 0; leg < 4; leg++) {
//        Vec3<float> f;
        for (int i = 0; i < 3; ++i) {
            f_ff[leg][i] = final_combined_force_robot_to_earth_robot_frame[leg * 3 + i];
        }
        f_ff_world_to_robot_ref[leg] = -seResult.rBody.transpose() * f_ff[leg];
//        f_ff[leg] = f;
        //        printf("[%d F:] %7.3f %7.3f %7.3f\n", leg, f_ff[leg][0], f_ff[leg][1],f_ff[leg][2]);
        // Update for WBC
        Fr_des[leg] = f_ff_world_to_robot_ref[leg];
    }

//    for (int leg = 0; leg < 4; ++leg) {
//        for (int i = 0; i < 3; ++i) {
//            f_ref_[leg*3 + i] = f_ff_world_to_robot_ref[leg][i];
//        }
//    }
}