#ifndef _KF_UTILS_H
#define _KF_UTILS_H

#include <Eigen/Geometry>
#include <queue>
#include <vector>
#include "const.h"

using namespace Eigen;
using namespace std;

enum LOCMODE{
    wheel = 0,
    DR = 1,
    Fused = 2,
    other = 3
};

enum KFSTATUS{
    UNINITIALIZED         = 0,
    ODOMETRY_INITIALIZING = 1,
    ODOMETRY_INITIALIZED  = 2,
    GLOBAL_COARSE_ALIGNMENT  =3,
    GLOBAL_FINE_ALIGNMENT   = 4,
    GLOBAL_FUSED          = 5,
    GNSS_OUTAGE           =6
};

float unwrap_diff(float new_angle, float previous_angle);
//Quaterniond quatMult(Quaterniond a, Quaterniond b);
Vector3d quat2rpy(Quaterniond q);
Quaterniond rpy2quat(const Vector3d& rpy);
Vector3d lla2ned(const Vector3d& lla, const Vector3d& lla0);
Vector3d lla2ned_naive(const Vector3d& lla, const Vector3d& lla0);
Matrix3d skew_symetric(const Vector3d& v);

template<typename T>
class query_queue
{
    public:
        query_queue():buffer_length(20){};
        query_queue(int len):buffer_length(len){};

        void push(const double& t, const T& v)
        {
            q.push({t, v});
            while(q.size() > buffer_length)
            {
                q.pop();
            }
        };

        void clear()
        {
            while(!q.empty())
            {
                q.pop();
            }
        };

        bool query(const double& t, T& v)
        {
            if(q.empty() || t<q.front().first) return false;

            double t1, t2;
            T v1, v2;

            while(!q.empty() && t > q.front().first)
            {
                t1 = q.front().first;
                v1 = q.front().second;
                q.pop();
            }

            if(q.empty()) return false;

            t2 = q.front().first;
            v2 = q.front().second;

            double interp;
            if(t2 - t1 < 0.005)
            {
                interp = 0;
            }
            else
            {
                interp = (t-t1)/(t2-t1);
            }

            v = v1 + interp * (v2 - v1);
            return true;
        };

    private:
        queue<pair<double, T>> q;
        int buffer_length;
};

#endif