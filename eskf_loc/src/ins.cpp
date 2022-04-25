#include "ins.h"

void InsStates::reset(){
    time = 0.0;

    C_a2b.setIdentity();
    C_g2b.setIdentity();
    C_ned2wgs.setIdentity();
    alt = 0;
    vel_ned.setZero();
    vel_ned_prev.setZero();
    Gv.setZero();
    Gv(2) = -9.81;
    d_theta_e.setZero();
    d_th.setZero();
    d_v.setZero();
    pos_wgs.setZero();
    ba.setZero();
    bg.setZero();
    Q_body2ned.setIdentity();
    ang_rate.setZero();
    acc_body.setZero();

    delta_dx = 0;
}

void InsStates::init(const InsParams& _params){
    ba = _params.ba;
    bg = _params.bg;
    C_a2b = _params.C_a2b;
    C_g2b = _params.C_g2b;

    local_gravity = _params.local_gravity;
    delta_dx = _params.delta_dx;

    Gv.setZero();
    Gv(2) = -local_gravity;
    side_velo = 0;
}

InsStates::InsStates(){
    reset();
}

InsStates::~InsStates(){

}

Imu InsStates::correctImu(const Imu& imu){

    Imu rtnVal;

    rtnVal.time = imu.time;
    Vector3d dvx = imu.dv - ba;
    Vector3d dthx = imu.dth - bg;
    dvx = C_a2b * dvx;
    dthx = C_g2b * dthx;

    rtnVal.dv = dvx;
    rtnVal.dth  = dthx;
    return rtnVal;
}

void InsStates::strapdown(const Imu& imu){
    Imu imuC = correctImu(imu);
    ang_rate = imuC.dth;
    Vector3d acc_meas = imuC.dv;

    double dt = imuC.time - time;

    if(dt < 0){
        return;
    }

    side_velo = imuC.dth(2) * delta_dx;

    imuC.dv = dt * imuC.dv;
    imuC.dth = dt * imuC.dth;
    time = imuC.time;

    d_v = d_v + imuC.dv;
    d_th = d_th + imuC.dth;

    Matrix3d C_body2ned;

    Quaterniond q_g(1.0, 0.5*imuC.dth(0), 0.5*imuC.dth(1), 0.5*imuC.dth(2));

    Q_body2ned = Q_body2ned *q_g;
    Q_body2ned.normalize();
    C_body2ned = Q_body2ned.toRotationMatrix();

    Vector3d dv_n = C_body2ned * imuC.dv;

    vel_ned = vel_ned + (dv_n - dt *Gv);
    pos_wgs = pos_wgs + vel_ned * dt;

    acc_body = acc_meas - C_body2ned.transpose() * Gv;

    odo_pos_buf.push(imuC.time,pos_wgs);
    odo_vel_buf.push(imuC.time,vel_ned);
    odo_euler_buf.push(imuC.time,quat2rpy(Q_body2ned));
}

void InsStates::correctIns(const Matrix<double, NUM_KF_STATES, 1>& filter_state){
    ba = Vector3d(filter_state[KF_STATE_ACCEL_BIAS_X], filter_state[KF_STATE_ACCEL_BIAS_Y], filter_state[KF_STATE_ACCEL_BIAS_Z]);
    bg = Vector3d(filter_state[KF_STATE_GYRO_BIAS_X], filter_state[KF_STATE_GYRO_BIAS_Y], filter_state[KF_STATE_GYRO_BIAS_Z]);

    Quaterniond dq_body2ned(1.0,
                            0.5 * filter_state[KF_STATE_MISALIGN_X],
                            0.5 * filter_state[KF_STATE_MISALIGN_Y],
                            0.5 * filter_state[KF_STATE_MISALIGN_Z]);

    Q_body2ned = dq_body2ned * Q_body2ned;

    Vector3d v = (vel_ned_prev + vel_ned) / 2.0;
    Vector3d phi(filter_state[KF_STATE_MISALIGN_X], filter_state[KF_STATE_MISALIGN_Y], filter_state[KF_STATE_MISALIGN_Z]);
    Vector3d dv_x(phi(1)*v(2) - phi(2)*v(1),
                 -phi(0)*v(2) + phi(2)*v(0),
                  phi(0)*v(1) - phi(1)*v(0));
    Vector3d dv(filter_state[KF_STATE_VELOCITY_X], filter_state[KF_STATE_VELOCITY_Y], filter_state[KF_STATE_VELOCITY_Z]);
    vel_ned = vel_ned + dv;

    Vector3d dp(filter_state[KF_STATE_POSITION_X], filter_state[KF_STATE_POSITION_Y], filter_state[KF_STATE_POSITION_Z]);
    pos_wgs = pos_wgs + dp;

    vel_ned_prev = vel_ned;
    d_th.setZero();
    d_v.setZero();   
}

