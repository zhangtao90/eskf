#ifndef _LOCALIZATION_H
#define _LOCALIZATION_H

#include <mutex>
#include <ros/ros.h>
#include <ros/console.h>

#include "rosmsg/GnssMsg.h"
#include "rosmsg/OdometerMsg.h"
#include "rosmsg/ImuMsg.h"
#include "rosmsg/LocalizationMsg.h"

#include "kalman_filter.h"
#include "localization_config.h"
#include "yaml-cpp/yaml.h"

class LocalizationHandler
{
    public:
        LocalizationHandler();
        ~LocalizationHandler();

        bool LoadConfigFile(std::string filename);
        void Config();

        void gnssCallback(const rosmsg::GnssMsg::ConstPtr& msg);
        void odometerCallback(const rosmsg::OdometerMsg::ConstPtr& msg);
        void imuCallback(const rosmsg::ImuMsg::ConstPtr& msg);
        void publishLocMsg();

        LocalizationConfig cfg;
        KalmanFilter loc_kf;

        ros::NodeHandle nh;
        ros::Subscriber sub_gnss;
        ros::Subscriber sub_odometer;
        ros::Subscriber sub_imu;

        ros::Publisher pub_loc;

        std::mutex m_loc;
};

#endif