#ifndef _INS_H
#define _INS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "kfstates.h"
#include "const.h"
#include "kf_utils.h"

using namespace Eigen;

struct InsParams{
    // imu instrincs...
    Vector3d ba;
    Vector3d bg;
    Matrix3d C_a2b;
    Matrix3d C_g2b;

    double local_gravity;
    double delta_dx;
};

struct Imu{
    double time;
    Vector3d dv;
    Vector3d dth;
};

class InsStates{
    public:
        InsStates();
        ~InsStates();

        void init(const InsParams& _params);
        void reset();
        Imu correctImu(const Imu& imu);

        void strapdown(const Imu& imu);
        void correctIns(const Matrix<double, NUM_KF_STATES, 1>&);

        double time;

        Matrix3d C_ned2wgs;
        Quaterniond Q_body2ned;
        Vector3d vel_ned;
        Vector3d Gv;
        Vector3d d_theta_e;
        Vector3d vel_ned_prev;
        Vector3d d_th;
        Vector3d d_v;
        Vector3d pos_wgs;
        Vector3d pos_wgs_prev;
        double alt;
        Vector3d ang_rate;
        Vector3d acc_body;

        Vector3d ba;
        Vector3d bg;
        Matrix3d C_a2b;
        Matrix3d C_g2b;

        double side_velo;

        InsParams m_param;

        double local_gravity;
        double delta_dx;

        query_queue<Vector3d> odo_pos_buf;
        query_queue<Vector3d> odo_vel_buf;
        query_queue<Vector3d> odo_euler_buf;

};

#endif