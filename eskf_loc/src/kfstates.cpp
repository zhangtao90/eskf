#include "kfstates.h"

KFStates::KFStates(): isInitialized(false), num_meas(0), num_constraints(0), time(-1.0)
{
    reset();
    init_P(m_params);
    init_Q(m_params);
}

KFStates::~KFStates() {};

void KFStates::reset()
{
    time = 0.0;

    for(int i = 0; i< NUM_KF_STATES; i++)
    {
        states(i) = 0.0;
        Q(i) = 0.0;
        for(int j = 0;j < 4; j++) Qd(i, j) = 0.0;
        for(int j = 0;j<NUM_KF_STATES; j++)
        {
            P(i, j) = 0.0;
            F(i, j) = 0.0;
        }
    }
}

void KFStates::init(const KFParams& params)
{
    init_P(m_params);
    init_Q(m_params);
    time = 0;
}

void KFStates::init_P(const KFParams& params)
{
    P.setZero(NUM_KF_STATES, NUM_KF_STATES);

    double temp = params.sigPos;
    double var = temp * temp;
    P(KF_STATE_POSITION_X , KF_STATE_POSITION_X) = var;
    P(KF_STATE_POSITION_Y , KF_STATE_POSITION_Y) = var;
    P(KF_STATE_POSITION_Z , KF_STATE_POSITION_Z) = var;

    temp = params.sigVel;
    var = temp * temp;
    P(KF_STATE_VELOCITY_X , KF_STATE_VELOCITY_X) = var;
    P(KF_STATE_VELOCITY_Y , KF_STATE_VELOCITY_Y) = var;
    P(KF_STATE_VELOCITY_Z , KF_STATE_VELOCITY_Z) = var;   

    P(KF_STATE_MISALIGN_X, KF_STATE_MISALIGN_X) = params.sig_tilt * params.sig_tilt; 
    P(KF_STATE_MISALIGN_Y, KF_STATE_MISALIGN_Y) = params.sig_tilt * params.sig_tilt; 
    P(KF_STATE_MISALIGN_Z, KF_STATE_MISALIGN_Z) = params.sig_alpha * params.sig_alpha; 

    temp = params.sigBg;
    var = temp * temp;
    P(KF_STATE_GYRO_BIAS_X, KF_STATE_GYRO_BIAS_X) = var;
    P(KF_STATE_GYRO_BIAS_Y, KF_STATE_GYRO_BIAS_Y) = var;
    P(KF_STATE_GYRO_BIAS_Z, KF_STATE_GYRO_BIAS_Z) = var;

    temp = params.sigBa;
    var = temp * temp;
    P(KF_STATE_ACCEL_BIAS_X, KF_STATE_ACCEL_BIAS_X) = var;
    P(KF_STATE_ACCEL_BIAS_Y, KF_STATE_ACCEL_BIAS_Y) = var;
    P(KF_STATE_ACCEL_BIAS_Z, KF_STATE_ACCEL_BIAS_Z) = var;
}

void KFStates::init_Q(const KFParams& params)
{
    double var;
    double rtq_pos, rtq_vel, rtq_tilt, rtq_acc, rtq_gyro;

    rtq_pos = params.rtq_pos;
    rtq_vel = params.awn;
    rtq_tilt = params.gwn;
    rtq_acc = params.acn;
    rtq_gyro = params.gcn;

    Qd.setZero(NUM_KF_STATES, 4);

    var = rtq_pos * rtq_pos;
    Qd(KF_STATE_POSITION_X, 0) = var;
    Qd(KF_STATE_POSITION_Y, 0) = var;
    Qd(KF_STATE_POSITION_Z, 0) = var;

    var = rtq_vel * rtq_vel;
    Qd(KF_STATE_VELOCITY_X, 0) = var;
    Qd(KF_STATE_VELOCITY_Y, 0) = var;
    Qd(KF_STATE_VELOCITY_Z, 0) = var;

    var = rtq_tilt * rtq_tilt;
    Qd(KF_STATE_MISALIGN_X, 0) = var;
    Qd(KF_STATE_MISALIGN_Y, 0) = var;
    Qd(KF_STATE_MISALIGN_Z, 0) = var;

    var = rtq_acc * rtq_acc;
    Qd(KF_STATE_ACCEL_BIAS_X, 0) = var;
    Qd(KF_STATE_ACCEL_BIAS_Y, 0) = var;
    Qd(KF_STATE_ACCEL_BIAS_Z, 0) = var;

    var = rtq_gyro * rtq_gyro;
    Qd(KF_STATE_GYRO_BIAS_X, 0) = var;
    Qd(KF_STATE_GYRO_BIAS_Y, 0) = var;
    Qd(KF_STATE_GYRO_BIAS_Z, 0) = var;
}

void KFStates::makeF(const Vector3d v, const Matrix3d c_b2n, const Vector3d fn)
{
    int i, j, ii, jj;

    F(KF_STATE_POSITION_X, KF_STATE_VELOCITY_X) = 1.0;
    F(KF_STATE_POSITION_Y, KF_STATE_VELOCITY_Y) = 1.0;
    F(KF_STATE_POSITION_Z, KF_STATE_VELOCITY_Z) = 1.0;

    F(KF_STATE_VELOCITY_X, KF_STATE_MISALIGN_X) = 0.0;
    F(KF_STATE_VELOCITY_X, KF_STATE_MISALIGN_Y) = fn(2);
    F(KF_STATE_VELOCITY_X, KF_STATE_MISALIGN_Z) = -fn(1);

    F(KF_STATE_VELOCITY_Y, KF_STATE_MISALIGN_X) = -fn(2);
    F(KF_STATE_VELOCITY_Y, KF_STATE_MISALIGN_Y) = 0.0;
    F(KF_STATE_VELOCITY_Y, KF_STATE_MISALIGN_Z) = fn(0);

    F(KF_STATE_VELOCITY_Z, KF_STATE_MISALIGN_X) = fn(1);
    F(KF_STATE_VELOCITY_Z, KF_STATE_MISALIGN_Y) = -fn(0);
    F(KF_STATE_VELOCITY_Z, KF_STATE_MISALIGN_Z) = 0.0;

    for(i = KF_STATE_VELOCITY_X, ii = 0; i<= KF_STATE_VELOCITY_Z; i++, ii++)
        for(j=KF_STATE_ACCEL_BIAS_X, jj = 0; j<= KF_STATE_ACCEL_BIAS_Z; j++, jj++)
            F(i, j) = -c_b2n(ii, jj);

    for(i = KF_STATE_MISALIGN_X, ii = 0; i<= KF_STATE_MISALIGN_Z; i++, ii++)
        for(j=KF_STATE_GYRO_BIAS_X, jj = 0; j<= KF_STATE_GYRO_BIAS_Z; j++, jj++)
            F(i, j) = -c_b2n(ii, jj);
}