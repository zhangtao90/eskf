#ifndef _LOCALIZATION_CONFIG_H
#define _LOCALIZATION_CONFIG_H

using namespace std;

class LocalizationConfig{
    public:

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

        double encoder_meas_stddev;
        double default_gnss_vel_stddev;
        double acc_deviation_th;
        double gyro_deviation_th;
        double spd_th;
        double window_tw;
        int least_imu_queue_length;
        int least_spd_queue_length;

        int align_vec_length;
        double align_min_delta_distance;
        double coarse_alignment_timeout;
        double fine_alignment_timeout;
        double gnss_outage_timeout;
        double odometry_converge_threshold_Pvel;
        double odometry_converge_threshold_Ptilt;
        double fine_alignment_converge_threshold_Ppos;
        double fine_alignment_converge_threshold_Pvel;
        double fine_alignment_converge_threshold_Pyaw;

        vector<double> C_a2b;
        vector<double> C_g2b;
        vector<double> ba;
        vector<double> bg;

        double local_gravity;
        double delta_dx;
        double wheel_sf;

};

#endif