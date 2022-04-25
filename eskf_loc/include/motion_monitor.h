#ifndef _MOTION_MONITOR_H_
#define _MOTION_MONITOR_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <deque>

#include <math.h>

#include "ins.h"
#include "kfstates.h"
#include "localization_config.h"

using namespace Eigen;
using namespace std;

enum MOTION_STATE{
    Mov = 0,
    NoMov = 1,
    Unknown = 2
};

struct MMParams{
    double acc_deviation_th;
    double gyro_deviation_th;
    double spd_th;
    double window_tw;
    int least_imu_queue_length;
    int least_spd_queue_length;
};

class MotionMonitor{
    public:
        MotionMonitor();
        ~MotionMonitor();
        void init(const MMParams&);
        void push_imu(const Imu& tImu);
        void push_spd(const double time, const double spd);
        MOTION_STATE CheckMotion();
        Vector3d GetGyroMean();
        Vector3d GetAccMean();
    private:
        MOTION_STATE mstate;
        MOTION_STATE update();
        queue<Imu> q_imu;
        queue<pair<double, double>> q_spd;

        double acc_var_th;
        double gyro_var_th;
        double spd_var_th;
        double window_tw;
        int least_imu_queue_length;
        int least_spd_queue_length;

        double gyro_x_sum;
        double gyro_y_sum;
        double gyro_z_sum;
        double acc_x_sum;
        double acc_y_sum;
        double acc_z_sum;

        double gyro_x_sqrsum;
        double gyro_y_sqrsum;
        double gyro_z_sqrsum;
        double acc_x_sqrsum;
        double acc_y_sqrsum;
        double acc_z_sqrsum;


        double gyro_x_var;
        double gyro_y_var;
        double gyro_z_var;
        double acc_x_var;
        double acc_y_var;
        double acc_z_var;

        double spd_sqrsum;
        double spd_var;
};

#endif