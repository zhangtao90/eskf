#include "motion_monitor.h"

MotionMonitor::MotionMonitor(): mstate(Unknown) {};

MotionMonitor::~MotionMonitor() {};

void MotionMonitor::init(const MMParams& params)
{
    acc_var_th = pow(params.acc_deviation_th,2);
    gyro_var_th = pow(params.gyro_deviation_th,2);
    spd_var_th = pow(params.spd_th,2);

    window_tw = params.window_tw;
    least_imu_queue_length = params.least_imu_queue_length;
    least_spd_queue_length = params.least_spd_queue_length;

    while(!q_imu.empty()) q_imu.pop();
    while(!q_spd.empty()) q_spd.pop();

    gyro_x_sum = 0;
    gyro_y_sum = 0;
    gyro_z_sum = 0;
    acc_x_sum = 0;
    acc_y_sum = 0;
    acc_z_sum = 0;

    gyro_x_sqrsum = 0;
    gyro_y_sqrsum = 0;
    gyro_z_sqrsum = 0;
    acc_x_sqrsum = 0;
    acc_y_sqrsum = 0;
    acc_z_sqrsum = 0;

    gyro_x_var = 0;
    gyro_y_var = 0;
    gyro_z_var = 0;
    acc_x_var = 0;
    acc_y_var = 0;
    acc_z_var = 0;  

    spd_sqrsum = 0;
    spd_var = 0;      
}

MOTION_STATE MotionMonitor::update()
{
    if(q_imu.size() < least_imu_queue_length || q_spd.size() < least_spd_queue_length)
    {
        return Unknown;
    } 

    gyro_x_var = (gyro_x_sqrsum/q_imu.size()) - pow(gyro_x_sum/q_imu.size(),2);
    gyro_y_var = (gyro_y_sqrsum/q_imu.size()) - pow(gyro_y_sum/q_imu.size(),2);
    gyro_z_var = (gyro_z_sqrsum/q_imu.size()) - pow(gyro_z_sum/q_imu.size(),2);
    acc_x_var = (acc_x_sqrsum/q_imu.size()) - pow(acc_x_sum/q_imu.size(),2);
    acc_y_var = (acc_y_sqrsum/q_imu.size()) - pow(acc_y_sum/q_imu.size(),2);
    acc_z_var = (acc_z_sqrsum/q_imu.size()) - pow(acc_z_sum/q_imu.size(),2);

    spd_var = spd_sqrsum/q_spd.size();

    return (
        gyro_x_var < gyro_var_th &&
        gyro_y_var < gyro_var_th &&
        gyro_z_var < gyro_var_th &&
        acc_x_var < acc_var_th &&
        acc_y_var < acc_var_th &&
        acc_z_var < acc_var_th &&
        spd_var < spd_var_th
    ) ? NoMov : Mov;
}

void MotionMonitor::push_imu(const Imu& tImu)
{
    double window_start_time = tImu.time - window_tw;

    while(!q_imu.empty() && q_imu.front().time < window_start_time)
    {
        gyro_x_sum -= q_imu.front().dth(0);
        gyro_y_sum -= q_imu.front().dth(1);
        gyro_z_sum -= q_imu.front().dth(2);
        acc_x_sum -= q_imu.front().dv(0);
        acc_y_sum -= q_imu.front().dv(1);
        acc_z_sum -= q_imu.front().dv(2);
        gyro_x_sqrsum -= pow(q_imu.front().dth(0), 2);
        gyro_y_sqrsum -= pow(q_imu.front().dth(1), 2);
        gyro_z_sqrsum -= pow(q_imu.front().dth(2), 2);
        acc_x_sqrsum -= pow(q_imu.front().dv(0), 2);
        acc_y_sqrsum -= pow(q_imu.front().dv(1), 2);
        acc_z_sqrsum -= pow(q_imu.front().dv(2), 2);
        q_imu.pop();
    }
    while(!q_spd.empty() && q_spd.front().first < window_start_time)
    {
        spd_sqrsum -= pow(q_spd.front().second,2);
        q_spd.pop();
    }

    gyro_x_sum += tImu.dth(0);
    gyro_y_sum += tImu.dth(1);
    gyro_z_sum += tImu.dth(2);
    acc_x_sum += tImu.dv(0);
    acc_y_sum += tImu.dv(1);
    acc_z_sum += tImu.dv(2);
    gyro_x_sqrsum += pow(tImu.dth(0),2);
    gyro_y_sqrsum += pow(tImu.dth(1),2);
    gyro_z_sqrsum += pow(tImu.dth(2),2);
    acc_x_sqrsum += pow(tImu.dv(0),2);
    acc_y_sqrsum += pow(tImu.dv(1),2);
    acc_z_sqrsum += pow(tImu.dv(2),2);
    q_imu.push(tImu);

    mstate = update();
}

void MotionMonitor::push_spd(const double time, const double spd)
{
    double window_start_time = time - window_tw;

    while(!q_imu.empty() && q_imu.front().time < window_start_time)
    {
        gyro_x_sum -= q_imu.front().dth(0);
        gyro_y_sum -= q_imu.front().dth(1);
        gyro_z_sum -= q_imu.front().dth(2);
        acc_x_sum -= q_imu.front().dv(0);
        acc_y_sum -= q_imu.front().dv(1);
        acc_z_sum -= q_imu.front().dv(2);
        gyro_x_sqrsum -= pow(q_imu.front().dth(0), 2);
        gyro_y_sqrsum -= pow(q_imu.front().dth(1), 2);
        gyro_z_sqrsum -= pow(q_imu.front().dth(2), 2);
        acc_x_sqrsum -= pow(q_imu.front().dv(0), 2);
        acc_y_sqrsum -= pow(q_imu.front().dv(1), 2);
        acc_z_sqrsum -= pow(q_imu.front().dv(2), 2);
        q_imu.pop();
    }
    while(!q_spd.empty() && q_spd.front().first < window_start_time)
    {
        spd_sqrsum -= pow(q_spd.front().second,2);
        q_spd.pop();
    }

    spd_sqrsum += pow(spd, 2);
    q_spd.push({time, spd});

    mstate = update();
}

MOTION_STATE MotionMonitor::CheckMotion()
{
    return mstate;
}

Vector3d MotionMonitor::GetGyroMean()
{
    if(q_imu.empty())
    {
        return Vector3d::Zero();
    }
    else
    {
        return Vector3d(gyro_x_sum/q_imu.size(), gyro_y_sum/q_imu.size(), gyro_z_sum/q_imu.size());
    }
}

Vector3d MotionMonitor::GetAccMean()
{
    if(q_imu.empty())
    {
        return Vector3d(0, 0, -9.81);
    }
    else
    {
        return Vector3d(acc_x_sum/q_imu.size(), acc_y_sum/q_imu.size(), acc_z_sum/q_imu.size());
    }
}