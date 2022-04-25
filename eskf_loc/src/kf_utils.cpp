#include "kf_utils.h"

float unwrap_diff(float new_angle, float previous_angle)
{
    float d = new_angle - previous_angle;
    d = d > PI ? d - 2 * PI : (d < -PI ? d + 2*PI : d);
    return d;
}

Vector3d quat2rpy(Quaterniond q)
{
    Vector3d ret;
    ret.setZero();

    q.normalize();
    ret(0) = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    ret(1) = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    ret(2) = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    return ret;
}

Quaterniond rpy2quat(const Vector3d& rpy)
{
    Quaterniond quat_yaw(AngleAxisd(rpy[2], Vector3d(0, 0, 1)));
    Quaterniond quat_pitch(AngleAxisd(rpy[1], Vector3d(0, 1, 0)));
    Quaterniond quat_roll(AngleAxisd(rpy[0], Vector3d(1, 0, 0)));

    return quat_yaw * quat_pitch * quat_roll;
}

Vector3d lla2ned(const Vector3d& lla, const Vector3d& lla0)
{
    double LAT_REF = lla0(0) * DEG2RAD;
    double LON_REF = lla0(1) * DEG2RAD;
    double ALT_REF = lla0(2);

    double latrad = lla(0) * DEG2RAD;
    double lonrad = lla(1) * DEG2RAD;
    double alt_= lla(2);

    // lla 2 ecef
    double slat = sin(latrad);
    double s2lat = slat * slat;
    double clat = cos(latrad);
    double slon = sin(lonrad);
    double clon = cos(lonrad);

    double rprime = EARTH_SEMIMAJOR / sqrt(1.0 - EARTH_ECCEN2 * s2lat);
    double ecefX = (rprime + alt_) * clat * clon; 
    double ecefY = (rprime + alt_) * clat * slon;
    double ecefZ = ((1.0 - EARTH_ECCEN2) * rprime + alt_) * slat;

    // lla ref 2 ecef ref
    double slat_ref = sin(LAT_REF);
    double s2lat_ref = slat_ref * slat_ref;
    double clat_ref = cos(LAT_REF);
    double slon_ref = sin(LON_REF);
    double clon_ref = cos(LON_REF);

    double rprime_ref = EARTH_SEMIMAJOR / sqrt(1.0 - EARTH_ECCEN2 * s2lat_ref);
    double ecefX_ref = (rprime_ref + ALT_REF) * clat_ref * clon_ref; 
    double ecefY_ref = (rprime_ref + ALT_REF) * clat_ref * slon_ref;
    double ecefZ_ref = ((1.0 - EARTH_ECCEN2) * rprime_ref + ALT_REF) * slat_ref;    

    // difference of ecef
    double dx = ecefX - ecefX_ref;
    double dy = ecefY - ecefY_ref;
    double dz = ecefZ - ecefZ_ref;

    // NED
    double north_ = -slat * clon * dx - slat * slon * dy + clat * dz;
    double east_ = -slon * dx + clon * dy;
    double down_ = -(clat * clon * dx + clat * slon * dy + slat * dz);

    return Vector3d(north_, east_, down_);
}

Vector3d lla2ned_naive(const Vector3d& lla, const Vector3d& lla0)
{
    double R = 6378137;
    double LAT_REF = lla0(0) * DEG2RAD;
    double LON_REF = lla0(1) * DEG2RAD;
    double ALT_REF = lla0(2);

    double latrad = lla(0) * DEG2RAD;
    double lonrad = lla(1) * DEG2RAD;
    double alt_= lla(2);

    double d_lat = latrad - LAT_REF;
    double avg_lat = (latrad + LAT_REF)/2;
    double d_lon = lonrad - LON_REF;

    double dh = alt_ - ALT_REF;

    double north_ = (R+alt_) * d_lat;
    double east_ = (R+alt_) * d_lon / cos(avg_lat);
    double down_ = -dh;

    return Vector3d(north_, east_, down_);
}

Matrix3d skew_symetric(const Vector3d& v)
{
    Matrix3d ret;
    ret << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return ret;
}