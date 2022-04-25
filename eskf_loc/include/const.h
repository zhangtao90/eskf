#ifndef __CONST_H
#define __CONST_H

#include <Eigen/Geometry>

#define D2R      1.74532925199433E-2
#define PI       3.1415926535898
#define EARTH_SEMIMAJOR     6378137.0
#define EARTH_ECCEN         0.08181919
#define EARTH_ECCEN2        0.00669438
#define DEG2RAD             0.01745329252
#define MSECONDS_PER_WEEK   (60*60*24*7*1000UL)

const double kv = 16.0;
const double imu_freq = 100.0;

#endif