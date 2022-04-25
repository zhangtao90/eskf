#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include "ins.h"
#include "kfstates.h"
#include "motion_monitor.h"
#include "kf_utils.h"

#include "localization_config.h"
#include "const.h"

using namespace std;
using namespace Eigen;

struct localization
{  
    double timestamp;
    int status;
    Vector3d posned;
    Vector3d velned;
    Vector3d euler;
    Quaterniond quat;
    Vector3d ang_rate;
    Vector3d acceleration;

    Matrix3d pos_cov;
    Matrix3d vel_cov;
    Matrix3d euler_cov;

};

class KalmanFilter{
    public:
        KalmanFilter();
        ~KalmanFilter();
        void init();
        void reset();
        bool predict(const InsStates& ins, const double& rTime);
        void ProcessWheelSpeed(double time, float measured_speed);
        void processImu(const Imu&);
        void processGnss(double time, const Vector3d& lla, const Vector3d vned, const Vector3d pacc, const Vector3d vacc, const bool valid);
        void config(const LocalizationConfig& config_file);
        bool isInitialized() {return kfstatus >= ODOMETRY_INITIALIZING;};
        double getInsTime() {return m_ins.time;};
        bool getMotionState() {return bNoMov;};
        void MotionMonitorProcess();

        bool CheckOdometryConvergence();
        bool GlobalCoarseAlignmentProcess_GN(double time, const Vector3d& pned, const Vector3d& vned);
        bool GlobalCoarseAlignmentProcess_SVD(double time, const Vector3d& pned, const Vector3d& vned);
        bool GlobalFineAlignmentProcess(double time);

        localization outputEgoStatesMsg(double t);

        KFParams m_kfparam;
        InsParams m_insparam;
        MMParams m_mmparam;

        KFStates m_kf;
        InsStates m_ins;

        KFSTATUS kfstatus;

        double gnss_heading;
        bool gnss_heading_valid;

    private:

        MotionMonitor m_mm;
        bool m_bOriginDetermined;

        int m_NumInnovFailure;
        double last_wheel_speed;
        double last_wheel_speed_time;

        double yaw_latch;
        Vector3d pos_latch;
        double yaw_var_latch;
        Matrix3d pos_var_latch;

        bool zhrWheelSpeed(double time, float measured_speed);
        bool zhrZupt();
        bool zhrGnss(double time, const Vector3d& pned, const Vector3d& vned, const Eigen::Vector3d& pacc, const Vector3d& vacc);
        bool zhrGnssHeading(double time, const Vector3d& vned);
        bool update(const VectorXd& z, const MatrixXd& H, const MatrixXd R, double kv);

        bool bNoMov, bNoMov_pre;

        double tNow;

        Vector3d lla0;
        double t_last_gnss_fix;

        vector<Vector3d> align_odo_pos;
        vector<Vector3d> align_glb_pos;
        vector<Vector3d> align_odo_vel;
        vector<Vector3d> align_glb_vel;

        Vector3d pos_odo;
        Vector3d vel_odo;

        int align_vec_length;
        double align_min_delta_distance;
        double coarse_alignment_timeout;
        double fine_alignment_timeout;
        double gnss_outage_timeout;

        double coarse_alignment_starttime;
        double fine_alignment_starttime;

        double odometry_converge_threshold_Pvel;
        double odometry_converge_threshold_Ptilt;
        double fine_alignment_converge_threshold_Ppos;
        double fine_alignment_converge_threshold_Pvel;
        double fine_alignment_converge_threshold_Pyaw;

};

#endif