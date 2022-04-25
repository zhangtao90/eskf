#include "localization.h"
#include <iostream>

LocalizationHandler::LocalizationHandler() {};
LocalizationHandler::~LocalizationHandler() {};

bool LocalizationHandler::LoadConfigFile(std::string filename)
{
    YAML::Node config_file_node = YAML::LoadFile(filename);

    if(config_file_node.IsNull())
    {
        return false;
    }

    cfg.C_a2b = config_file_node["C_a2b"]["data"].as<std::vector<double>>();
    cfg.C_g2b = config_file_node["C_g2b"]["data"].as<std::vector<double>>();
    cfg.ba = config_file_node["ba"]["data"].as<std::vector<double>>();
    cfg.bg = config_file_node["bg"]["data"].as<std::vector<double>>();
    cfg.delta_dx = config_file_node["delta_dx"].as<double>();
    cfg.wheel_sf = config_file_node["wheel_sf"].as<double>();
    cfg.encoder_meas_stddev = config_file_node["encoder_meas_stddev"].as<double>();
    cfg.default_gnss_vel_stddev = config_file_node["default_gnss_vel_stddev"].as<double>();

    cfg.sigPos = config_file_node["sigPos"].as<double>();
    cfg.sigVel = config_file_node["sigVel"].as<double>();
    cfg.sig_tilt = config_file_node["sig_tilt"].as<double>();
    cfg.sig_alpha = config_file_node["sig_alpha"].as<double>();
    cfg.sigBa = config_file_node["sigBa"].as<double>();
    cfg.sigBg = config_file_node["sigBg"].as<double>();
    cfg.rtq_pos = config_file_node["rtq_pos"].as<double>();
    cfg.awn = config_file_node["awn"].as<double>();
    cfg.gwn = config_file_node["gwn"].as<double>();
    cfg.acn = config_file_node["acn"].as<double>();
    cfg.gcn = config_file_node["gcn"].as<double>();
    cfg.ac_tau = config_file_node["ac_tau"].as<double>();
    cfg.gc_tau = config_file_node["gc_tau"].as<double>();

    cfg.align_vec_length = config_file_node["align_vec_length"].as<int>();
    cfg.align_min_delta_distance = config_file_node["align_min_delta_distance"].as<double>();
    cfg.coarse_alignment_timeout = config_file_node["coarse_alignment_timeout"].as<double>();
    cfg.fine_alignment_timeout = config_file_node["fine_alignment_timeout"].as<double>();
    cfg.gnss_outage_timeout = config_file_node["gnss_outage_timeout"].as<double>();

    cfg.odometry_converge_threshold_Pvel = config_file_node["odometry_converge_threshold_Pvel"].as<int>();
    cfg.odometry_converge_threshold_Ptilt = config_file_node["odometry_converge_threshold_Ptilt"].as<double>();
    cfg.fine_alignment_converge_threshold_Ppos = config_file_node["fine_alignment_converge_threshold_Ppos"].as<double>();
    cfg.fine_alignment_converge_threshold_Pvel = config_file_node["fine_alignment_converge_threshold_Pvel"].as<double>();
    cfg.fine_alignment_converge_threshold_Pyaw = config_file_node["fine_alignment_converge_threshold_Pyaw"].as<double>();    

    cfg.local_gravity = config_file_node["local_gravity"].as<double>();    

    cfg.acc_deviation_th = config_file_node["acc_deviation_th"].as<double>();    
    cfg.gyro_deviation_th = config_file_node["gyro_deviation_th"].as<double>();    
    cfg.spd_th = config_file_node["spd_th"].as<double>();    
    cfg.window_tw = config_file_node["window_tw"].as<double>();    
    cfg.least_imu_queue_length = config_file_node["least_imu_queue_length"].as<int>();
    cfg.least_spd_queue_length = config_file_node["least_spd_queue_length"].as<int>();

    return true;
}

void LocalizationHandler::Config()
{
    loc_kf.config(cfg);

    sub_gnss = nh.subscribe("/gnss", 10, &LocalizationHandler::gnssCallback, this);
    sub_imu = nh.subscribe("/imu", 100, &LocalizationHandler::imuCallback, this);
    sub_odometer = nh.subscribe("/odometer", 50, &LocalizationHandler::odometerCallback, this);

    pub_loc = nh.advertise<rosmsg::LocalizationMsg>("/localization", 100);
}

void LocalizationHandler::gnssCallback(const rosmsg::GnssMsg::ConstPtr& msg)
{

}

void LocalizationHandler::odometerCallback(const rosmsg::OdometerMsg::ConstPtr& msg)
{

}

void LocalizationHandler::imuCallback(const rosmsg::ImuMsg::ConstPtr& msg)
{

}

void LocalizationHandler::publishLocMsg()
{

}

int main(int argc, char** argv)
{

    if(argc != 2)
    {
        std::cout << "need config file name as one argument !" << std::endl;
        return 1;
    }

    std::string filename(argv[1]);

    ros::init(argc, argv, "localization");

    LocalizationHandler loc_handler;
    if(!loc_handler.LoadConfigFile(filename))
    {
        std::cout << "load config file fail !" << std::endl;
        return 1;
    }

    loc_handler.Config();

    ros::spin();

    return 0;
}