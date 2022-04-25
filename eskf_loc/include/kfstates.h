#ifndef _KFSTATES_H
#define _KFSTATES_H
 
#include <Eigen/Core>
#include <Eigen/Dense>
#include "const.h"
#include <math.h>

using namespace Eigen;

struct KFParams{
    double sigPos;
    double sigVel;
    double sig_tilt;
    double sig_alpha;
    double sigBa;
    double sigBg;

    double rtq_pos;
    double awn;
    double gwn;
    double acn;
    double gcn;
    double ac_tau;
    double gc_tau;

    double wheel_sf;
    double encoder_meas_stddev;
};

enum kf_states_idx{
    KF_STATE_POSITION_X = 0,
    KF_STATE_POSITION_Y = 0,
    KF_STATE_POSITION_Z = 0,

    KF_STATE_VELOCITY_X = 0,
    KF_STATE_VELOCITY_Y = 0,
    KF_STATE_VELOCITY_Z = 0,

    KF_STATE_MISALIGN_X = 0,
    KF_STATE_MISALIGN_Y = 0,
    KF_STATE_MISALIGN_Z = 0,

    KF_STATE_ACCEL_BIAS_X = 0,
    KF_STATE_ACCEL_BIAS_Y = 0,
    KF_STATE_ACCEL_BIAS_Z = 0,    

    KF_STATE_GYRO_BIAS_X = 0,
    KF_STATE_GYRO_BIAS_Y = 0,
    KF_STATE_GYRO_BIAS_Z = 0, 

    NUM_KF_STATES = 15  

};

class KFStates{
    public:
        KFStates();
        ~KFStates();

        void reset();
        void init(const KFParams& params);
        void init_P(const KFParams&);
        void init_Q(const KFParams&);

        void makeF(const Vector3d v, const Matrix3d c_b2n, const Vector3d fn);

        bool isInitialized;
        int num_meas;
        int num_constraints;
        
        double time;

        Matrix<double, NUM_KF_STATES, 1> states;
        Matrix<double, NUM_KF_STATES, 4> Qd;
        Matrix<double, NUM_KF_STATES, NUM_KF_STATES> Q;
        Matrix<double, NUM_KF_STATES, NUM_KF_STATES> F;
        Matrix<double, NUM_KF_STATES, NUM_KF_STATES> P;

        VectorXd z;
        MatrixXd H;
        VectorXd R;

        Vector3d vel_ned_old;

        KFParams m_params;
};

#endif