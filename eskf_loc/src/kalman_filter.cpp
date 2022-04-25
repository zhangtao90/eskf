#include "kalman_filter.h"

KalmanFilter::KalmanFilter(){
    reset();
};

KalmanFilter::~KalmanFilter(){
    reset();
};

void KalmanFilter::config(const LocalizationConfig& cfg){
    m_kfparam.sigPos = cfg.sigPos;
    m_kfparam.sigVel = cfg.sigVel;
    m_kfparam.sig_tilt = cfg.sig_tilt;
    m_kfparam.sig_alpha = cfg.sig_alpha;
    m_kfparam.sigBa = cfg.sigBa;
    m_kfparam.sigBg = cfg.sigBg;
    m_kfparam.rtq_pos = cfg.rtq_pos;
    m_kfparam.awn = cfg.awn;
    m_kfparam.gwn = cfg.gwn;
    m_kfparam.acn = cfg.acn;
    m_kfparam.gcn = cfg.gcn;
    m_kfparam.ac_tau = cfg.ac_tau;
    m_kfparam.gc_tau = cfg.gc_tau;
    m_kfparam.wheel_sf = cfg.wheel_sf;
    m_kfparam.encoder_meas_stddev = cfg.encoder_meas_stddev;

    m_kfparam.sig_tilt = m_kfparam.sig_tilt * D2R;
    m_kfparam.sig_alpha = m_kfparam.sig_alpha * D2R;
    m_kfparam.sigBa = m_kfparam.sigBa * 1e-3 * 9.81;
    m_kfparam.sigBg = m_kfparam.sigBg * D2R / 3600;
    m_kfparam.awn = m_kfparam.awn * 1e-6 *9.81;
    m_kfparam.gwn = m_kfparam.gwn * D2R /60;    
    m_kfparam.acn = m_kfparam.acn * 1e-3 *9.81 * sqrt(2/m_kfparam.ac_tau);
    m_kfparam.gcn = m_kfparam.gcn * D2R /3600 * sqrt(2/m_kfparam.gc_tau);

    m_insparam.C_a2b = Matrix3d(cfg.C_a2b.data());
    m_insparam.C_g2b = Matrix3d(cfg.C_g2b.data());
    m_insparam.ba = Vector3d(cfg.ba.data());
    m_insparam.bg = Vector3d(cfg.bg.data());
    m_insparam.local_gravity = cfg.local_gravity;
    m_insparam.delta_dx = cfg.delta_dx;

    m_mmparam.acc_deviation_th = cfg.acc_deviation_th;
    m_mmparam.gyro_deviation_th = cfg.gyro_deviation_th;
    m_mmparam.spd_th = cfg.spd_th;
    m_mmparam.window_tw = cfg.window_tw;
    m_mmparam.least_imu_queue_length = cfg.least_imu_queue_length;
    m_mmparam.least_spd_queue_length = cfg.least_spd_queue_length;

    align_vec_length = cfg.align_vec_length;
    align_min_delta_distance = cfg.align_min_delta_distance;
    coarse_alignment_timeout = cfg.coarse_alignment_timeout;
    fine_alignment_timeout = cfg.fine_alignment_timeout;
    gnss_outage_timeout = cfg.gnss_outage_timeout;

    odometry_converge_threshold_Pvel = cfg.odometry_converge_threshold_Pvel;
    odometry_converge_threshold_Ptilt = cfg.odometry_converge_threshold_Ptilt;
    fine_alignment_converge_threshold_Ppos = cfg.fine_alignment_converge_threshold_Ppos;
    fine_alignment_converge_threshold_Pvel = cfg.fine_alignment_converge_threshold_Pvel;
    fine_alignment_converge_threshold_Pyaw = cfg.fine_alignment_converge_threshold_Pyaw;

    odometry_converge_threshold_Ptilt = odometry_converge_threshold_Ptilt * D2R;
    odometry_converge_threshold_Pvel = odometry_converge_threshold_Pvel * D2R;

    odometry_converge_threshold_Pvel = odometry_converge_threshold_Pvel*odometry_converge_threshold_Pvel;
    odometry_converge_threshold_Ptilt = odometry_converge_threshold_Ptilt*odometry_converge_threshold_Ptilt;
    fine_alignment_converge_threshold_Ppos = fine_alignment_converge_threshold_Ppos*fine_alignment_converge_threshold_Ppos;
    fine_alignment_converge_threshold_Pvel = fine_alignment_converge_threshold_Pvel*fine_alignment_converge_threshold_Pvel;
    fine_alignment_converge_threshold_Pyaw = fine_alignment_converge_threshold_Pyaw*fine_alignment_converge_threshold_Pyaw;    

    init();
}

void KalmanFilter::reset(){
    last_wheel_speed = -1.0;
    last_wheel_speed_time = 0.0;

    yaw_latch = 0;
    pos_latch.setZero();
    yaw_var_latch = 0;
    pos_var_latch.setZero();
    t_last_gnss_fix = 0;
    kfstatus = UNINITIALIZED;
    m_bOriginDetermined = false;
    align_vec_length = 4;
    align_min_delta_distance = 5;
    coarse_alignment_timeout = 30;
    fine_alignment_timeout = 30;
    gnss_outage_timeout = 120;
}

void KalmanFilter::init()
{
    m_kf.init(m_kfparam);
    m_ins.init(m_insparam);
    m_mm.init(m_mmparam);
    kfstatus = UNINITIALIZED;
    gnss_heading = 0;
    gnss_heading_valid = false;
    bNoMov = false;
    bNoMov_pre = false;
}

bool KalmanFilter::predict(const InsStates& ins, const double& time)
{
    if(m_kf.time == 0.0)
    {
        m_kf.time = time;
    }

    double t_prop = (time - m_kf.time);

    if(t_prop < 1e-4)
    {
        return false;
    }

    Vector3d v_mid = (m_ins.vel_ned + m_ins.vel_ned_prev) * 0.5;
    Vector3d awa_vec = (m_ins.vel_ned - m_ins.vel_ned_prev) / t_prop + Vector3d(0,0,-m_insparam.local_gravity);
    Vector3d wib_vec = m_ins.d_th/t_prop;

    m_ins.Q_body2ned.normalize();

    Matrix3d c_b2n = m_ins.Q_body2ned.toRotationMatrix();
    Vector3d dphi(m_kf.states[KF_STATE_MISALIGN_X], m_kf.states[KF_STATE_MISALIGN_Y],m_kf.states[KF_STATE_MISALIGN_Z]);

    m_kf.makeF(v_mid, c_b2n, awa_vec);

    MatrixXd I = MatrixXd::Identity(NUM_KF_STATES, NUM_KF_STATES);
    Matrix<double, NUM_KF_STATES, NUM_KF_STATES> phi = I + m_kf.F * t_prop;

    Matrix<double, NUM_KF_STATES,NUM_KF_STATES> P_minus = phi * m_kf.P;

    P_minus = P_minus * phi.transpose();

    double accel = awa_vec.norm();

    double ag = sqrt(c_b2n(2, 0)*c_b2n(2, 0) + c_b2n(2, 1)*c_b2n(2, 1)) * m_insparam.local_gravity;
    accel += ag;

    double vg = sqrt(v_mid(0)*v_mid(0) + v_mid(1) * v_mid(1));

    double wib = wib_vec.norm();

    m_kf.Q.setZero();

    for(int i = 0;i< NUM_KF_STATES ; i++)
    {
        m_kf.Q(i, i) = m_kf.Qd(i, 0);
    }

    m_kf.P = P_minus + m_kf.Q * t_prop;

    m_kf.time = time;

    return true;

}

bool KalmanFilter::update(const VectorXd& z, const MatrixXd& H, const MatrixXd R, double kv)
{
    bool bUpdated = false;
    m_kf.states.setZero();

    for(int iz = 0;iz < z.size();iz++)
    {
        auto Hi = H.row(iz);
        double z_hat = Hi.dot(m_kf.states);
        double y = z(iz);
        auto Ht = Hi.transpose();
        auto PHt = m_kf.P * Ht;
        double S = Hi.dot(PHt) + R(iz, iz);
    

        if( kv == 0)
        {
            bUpdated = true;
        }
        else
        {
            bool bPass = true;
            if( S < 1e-10 || y * y > kv * S) bPass = false;
            m_NumInnovFailure = bPass ? m_NumInnovFailure : m_NumInnovFailure + 1;
            bUpdated = bPass;
        }

        if(bUpdated)
        {
            double Si = 1.0/S;
            auto K = PHt * Si;

            auto dx = K * y;

            m_kf.states = m_kf.states + dx;

            MatrixXd I = MatrixXd::Identity(NUM_KF_STATES, NUM_KF_STATES);
            m_kf.P = (I - K * Hi) * m_kf.P;
        }
    }

    return bUpdated;
}

void KalmanFilter::MotionMonitorProcess()
{
    bNoMov_pre = bNoMov;
    bNoMov = (m_mm.CheckMotion() == NoMov);

    if(bNoMov && (!bNoMov_pre))
    {
        auto euler = quat2rpy(m_ins.Q_body2ned);
        yaw_latch = euler(2);
        pos_latch = m_ins.C_ned2wgs.transpose() * m_ins.pos_wgs;
        yaw_var_latch = m_kf.P(KF_STATE_MISALIGN_Z, KF_STATE_MISALIGN_Z);
        pos_var_latch = m_kf.P.block<3,3>(KF_STATE_POSITION_X,KF_STATE_POSITION_X);
    }
    else if((!bNoMov) && bNoMov_pre)
    {
        m_kf.P(KF_STATE_MISALIGN_Z, KF_STATE_MISALIGN_Z) = yaw_var_latch;
        m_kf.P.block<3,3>(KF_STATE_POSITION_X,KF_STATE_POSITION_X) = pos_var_latch;        
    }
}

void KalmanFilter::ProcessWheelSpeed(double time, float measured_speed)
{
    m_mm.push_spd(time, measured_speed);
    MotionMonitorProcess();

    if(kfstatus == UNINITIALIZED)
    {
        Matrix3d C_b2n = m_ins.Q_body2ned.toRotationMatrix();
        Vector3d v_b;

        v_b.setZero();
        v_b(0) = measured_speed;
        v_b(1) = m_ins.side_velo;
        
        Vector3d v_n = C_b2n * v_b;
        m_ins.vel_ned_prev = v_n;
        m_ins.vel_ned_prev = v_n;
        kfstatus = ODOMETRY_INITIALIZING;
    }
    else
    {
        if(kfstatus == ODOMETRY_INITIALIZING)
        {
            if(CheckOdometryConvergence() && !bNoMov)
            {
                kfstatus = ODOMETRY_INITIALIZED;
            }
        }
        if(predict(m_ins, time))
        {
            if(bNoMov)
            {
                if(zhrZupt())
                {
                    m_ins.correctIns(m_kf.states);
                }
            }
            else
            {
                if(zhrWheelSpeed(time, measured_speed))
                {
                    m_ins.correctIns(m_kf.states);
                }
            }
        }
    }

    last_wheel_speed = measured_speed;
    last_wheel_speed_time = time;
}

bool KalmanFilter::CheckOdometryConvergence()
{
    return m_kf.P(KF_STATE_VELOCITY_X, KF_STATE_VELOCITY_X) < odometry_converge_threshold_Pvel &&
           m_kf.P(KF_STATE_VELOCITY_Y, KF_STATE_VELOCITY_Y) < odometry_converge_threshold_Pvel &&
           m_kf.P(KF_STATE_VELOCITY_Z, KF_STATE_VELOCITY_Z) < odometry_converge_threshold_Pvel &&
           m_kf.P(KF_STATE_MISALIGN_X, KF_STATE_MISALIGN_X) < odometry_converge_threshold_Ptilt &&
           m_kf.P(KF_STATE_MISALIGN_Y, KF_STATE_MISALIGN_Y) < odometry_converge_threshold_Ptilt;
}

bool KalmanFilter::zhrWheelSpeed(double time, float measured_speed)
{
    Matrix3d C_b2n = m_ins.Q_body2ned.toRotationMatrix();
    Vector3d v_b;

    v_b.setZero();
    v_b(0) = measured_speed;
    v_b(1) = m_ins.side_velo;
    
    Vector3d v_n = C_b2n * v_b;

    VectorXd z = v_n - m_ins.vel_ned;

    MatrixXd H;
    H.resize(3, NUM_KF_STATES);
    H.setZero();
    H(0, KF_STATE_VELOCITY_X) = 1.0;
    H(0, KF_STATE_VELOCITY_Y) = 1.0;
    H(0, KF_STATE_VELOCITY_Z) = 1.0;

    MatrixXd R;
    R.resize(3, 3);
    R.setZero();

    double v_r = m_kfparam.encoder_meas_stddev * m_kfparam.encoder_meas_stddev;
    R(0, 0) = v_r;
    R(1, 1) = v_r;
    R(2, 2) = v_r;

    R = C_b2n * R * C_b2n.transpose();

    Matrix3d L = R.llt().matrixL();

    z = L.inverse() * z;
    H = L.inverse() * H;
    R.setIdentity();

    if(!update(z, H, R, kv))
    {
        return false;
    }

    return true;

}

void KalmanFilter::processGnss(double time, const Vector3d& lla, const Vector3d vned, const Vector3d pacc, const Vector3d vacc, const bool valid)
{
    Vector3d pned;
    double curtime = time;

    if(valid)
    {
        if(!m_bOriginDetermined)
        {
            lla0 = lla;
            m_bOriginDetermined = true;
        }
        pned = lla2ned(lla, lla0);
        t_last_gnss_fix - curtime;
    }

    if(kfstatus < ODOMETRY_INITIALIZED)
    {
        return ;
    }
    else if(kfstatus == ODOMETRY_INITIALIZED)
    {
        if(valid)
        {
            align_odo_pos.clear();
            align_glb_pos.clear();
            align_odo_vel.clear();
            align_glb_vel.clear();
            coarse_alignment_starttime = curtime;
            kfstatus = GLOBAL_COARSE_ALIGNMENT;
        }
    }
    else if(kfstatus == GLOBAL_COARSE_ALIGNMENT)
    {
        if(curtime - coarse_alignment_starttime > coarse_alignment_timeout)
        {
            kfstatus = ODOMETRY_INITIALIZED;
            return ;
        }

        if(valid)
        {
            if(GlobalCoarseAlignmentProcess_SVD(time, pned, vned))
            {
                fine_alignment_starttime = curtime;
                kfstatus = GLOBAL_FINE_ALIGNMENT;
            }
        }
    }
    else if(kfstatus >= GLOBAL_FINE_ALIGNMENT)
    {
        if(kfstatus == GLOBAL_FINE_ALIGNMENT)
        {
            if(curtime - fine_alignment_starttime > fine_alignment_timeout)
            {
                kfstatus = ODOMETRY_INITIALIZED;
                return;
            }
            else if(GlobalFineAlignmentProcess(time))
            {
                kfstatus = GLOBAL_FUSED;
            }
        }
        else if(kfstatus == GLOBAL_FUSED)
        {
            if(curtime - t_last_gnss_fix > 3)
            {
                kfstatus = GNSS_OUTAGE;
            }
        }
        else if(kfstatus == GNSS_OUTAGE)
        {
            if(curtime - t_last_gnss_fix < 3)
            {
                kfstatus = GLOBAL_FUSED;
            }
            else if(curtime - t_last_gnss_fix > gnss_outage_timeout)
            {
                kfstatus = ODOMETRY_INITIALIZED;
            }
        }

        if(valid && (!bNoMov))
        {
            if(zhrGnss(time, pned, vned, pacc, vacc))
            {
                m_ins.correctIns(m_kf.states);
            }
            if(zhrGnssHeading(time,vned))
            {
                m_ins.correctIns(m_kf.states);
            }
        }
    }
}

bool KalmanFilter::GlobalCoarseAlignmentProcess_GN(double time, const Vector3d& pned, const Vector3d& vned)
{
    Vector3d pos_odo;
    Vector3d vel_odo;

    if(!m_ins.odo_pos_buf.query(time, pos_odo))
    {
        return false;
    }
    m_ins.odo_vel_buf.query(time, vel_odo);

    if(align_odo_pos.empty() || (align_odo_pos.size() < align_vec_length && (pos_odo - align_odo_pos.back()).norm() > align_min_delta_distance) )
    {
        align_odo_pos.push_back(pos_odo);
        align_glb_pos.push_back(pned);
        return false;
    } 
    else if(align_odo_pos.size() == align_vec_length)
    {
        //guess initial qGL
        Vector2d dpxy_glb(align_glb_pos.back()(0)-align_glb_pos.front()(0),align_glb_pos.back()(1)-align_glb_pos.front()(1));
        Vector2d dpxy_odo(align_odo_pos.back()(0)-align_odo_pos.front()(0),align_odo_pos.back()(1)-align_odo_pos.front()(1));
    
        Matrix2d odo_cross;
        odo_cross << dpxy_odo(0) , - dpxy_odo(1) , dpxy_odo(1) , dpxy_odo(0);
        Eigen::Vector2d cs_yaw = (odo_cross.inverse() * dpxy_glb).normalized();
        double delta_yaw = atan2(cs_yaw(1), cs_yaw(0));
        Quaterniond q_GL = rpy2quat(Vector3d(0,0,delta_yaw));

        //guess initial tGL
        Vector3d t_GL = align_glb_pos[0] - q_GL * align_odo_pos[0];
        int iterationcnt = 0;

        //gauss newton iteration
        while(1)
        {
            if(iterationcnt >= 10)
            {
                break;
            }

            //residual
            VectorXd e(3*align_vec_length);

            for(int i = 0;i<align_vec_length;i++)
            {
                e.segment<3>(3*i) = q_GL * align_odo_pos[i] + t_GL - align_glb_pos[i];
            }

            if(e.norm()/e.size() < 0.001)
            {
                break;
            }

            //jacobian
            MatrixXd J(3*align_vec_length, 4);

            for(int i = 0;i<align_vec_length;i++)
            {
                Matrix3d J_e_phi = - q_GL.toRotationMatrix() * skew_symetric(align_odo_pos[i]);
                J.block<3,1>(3*i, 0) = J_e_phi.block<3,1>(0,2);
                J.block<3,3>(3*i, 1) = Matrix3d::Identity(); 
            }

            Vector4d dx = -(J.transpose()*J).inverse() * J.transpose() * e;

            Quaterniond dq = rpy2quat(Vector3d(0,0,dx(0)));
            q_GL = q_GL * dq;
            t_GL = t_GL + dx.segment<3>(1);

            iterationcnt ++;
        }

        Matrix3d R_GL = q_GL.toRotationMatrix();

        m_ins.pos_wgs = R_GL * m_ins.pos_wgs + t_GL;
        m_ins.pos_wgs_prev = R_GL * m_ins.pos_wgs_prev + t_GL;

        m_ins.vel_ned = R_GL * m_ins.vel_ned;
        m_ins.vel_ned_prev = R_GL * m_ins.vel_ned_prev;

        m_ins.Q_body2ned = q_GL * m_ins.Q_body2ned;

        m_kf.P(KF_STATE_POSITION_X, KF_STATE_POSITION_X) = 1;
        m_kf.P(KF_STATE_POSITION_Y, KF_STATE_POSITION_Y) = 1;
        m_kf.P(KF_STATE_POSITION_Z, KF_STATE_POSITION_Z) = 1;

        m_kf.P(KF_STATE_VELOCITY_X, KF_STATE_VELOCITY_X) = 0.3;
        m_kf.P(KF_STATE_VELOCITY_Y, KF_STATE_VELOCITY_Y) = 0.3;

        m_kf.P(KF_STATE_MISALIGN_Z, KF_STATE_MISALIGN_Z) = 2e-3;

        return true;

    }
}

bool KalmanFilter::GlobalCoarseAlignmentProcess_SVD(double time, const Vector3d& pned, const Vector3d& vned)
{
    Vector3d pos_odo;
    Vector3d vel_odo;

    if(!m_ins.odo_pos_buf.query(time, pos_odo))
    {
        return false;
    }
    m_ins.odo_vel_buf.query(time, vel_odo);

    if(align_odo_pos.empty() || (align_odo_pos.size() < align_vec_length && (pos_odo - align_odo_pos.back()).norm() > align_min_delta_distance) )
    {
        align_odo_pos.push_back(pos_odo);
        align_glb_pos.push_back(pned);
        align_odo_vel.push_back(vel_odo);
        align_glb_vel.push_back(vned);
        return false;
    }     
    else if(align_odo_pos.size() == align_vec_length)
    {
        Vector3d pos_odo_avrg, pos_glb_avrg, vel_odo_avrg, vel_glb_avrg;
        pos_odo_avrg.setZero();
        pos_glb_avrg.setZero();
        vel_odo_avrg.setZero();
        vel_glb_avrg.setZero();

        for(int i = 0;i<align_vec_length;i++)
        {
            pos_odo_avrg += align_odo_pos[i];
            pos_glb_avrg += align_glb_pos[i];
        }

        pos_odo_avrg /= double(align_vec_length);
        pos_glb_avrg /= double(align_vec_length);

        for(int i = 0;i<align_vec_length;i++)
        {
            align_odo_pos[i] -= pos_odo_avrg;
            align_glb_pos[i] -= pos_glb_avrg;
        }

        Matrix2d w;

        for(int i = 0;i<align_vec_length;i++)
        {
            w += align_glb_pos[i].head(2) * align_odo_pos[i].head(2).transpose();
            w += align_glb_vel[i].head(2) * align_odo_vel[i].head(2).transpose();
        }

        JacobiSVD<Matrix2d> svd(w, ComputeFullU | ComputeFullV);

        auto U = svd.matrixU();
        auto V = svd.matrixV();

        Matrix3d R_GL;
        R_GL.setIdentity();
        R_GL.block<2,2>(0,0) = U*V.transpose();
        Vector3d t_GL = pos_glb_avrg - R_GL * pos_odo_avrg;

        Quaterniond q_GL = Quaterniond(R_GL);

        m_ins.pos_wgs = R_GL * m_ins.pos_wgs + t_GL;
        m_ins.pos_wgs_prev = R_GL * m_ins.pos_wgs_prev + t_GL;

        m_ins.vel_ned = R_GL * m_ins.vel_ned;
        m_ins.vel_ned_prev = R_GL * m_ins.vel_ned_prev;

        m_ins.Q_body2ned = q_GL * m_ins.Q_body2ned;

        m_kf.P(KF_STATE_POSITION_X, KF_STATE_POSITION_X) = 1;
        m_kf.P(KF_STATE_POSITION_Y, KF_STATE_POSITION_Y) = 1;
        m_kf.P(KF_STATE_POSITION_Z, KF_STATE_POSITION_Z) = 1;

        m_kf.P(KF_STATE_VELOCITY_X, KF_STATE_VELOCITY_X) = 0.3;
        m_kf.P(KF_STATE_VELOCITY_Y, KF_STATE_VELOCITY_Y) = 0.3;

        m_kf.P(KF_STATE_MISALIGN_Z, KF_STATE_MISALIGN_Z) = 2e-3;

        return true;

    }
}

bool KalmanFilter::GlobalFineAlignmentProcess(double time)
{
    return m_kf.P(KF_STATE_VELOCITY_X, KF_STATE_VELOCITY_X) < fine_alignment_converge_threshold_Pvel &&
           m_kf.P(KF_STATE_VELOCITY_Y, KF_STATE_VELOCITY_Y) < fine_alignment_converge_threshold_Pvel &&
           m_kf.P(KF_STATE_VELOCITY_Z, KF_STATE_VELOCITY_Z) < fine_alignment_converge_threshold_Pvel &&
           m_kf.P(KF_STATE_POSITION_X, KF_STATE_POSITION_X) < fine_alignment_converge_threshold_Ppos &&
           m_kf.P(KF_STATE_POSITION_Y, KF_STATE_POSITION_Y) < fine_alignment_converge_threshold_Ppos &&
           m_kf.P(KF_STATE_POSITION_Z, KF_STATE_POSITION_Z) < fine_alignment_converge_threshold_Ppos &&
           m_kf.P(KF_STATE_MISALIGN_Z, KF_STATE_MISALIGN_Z) < fine_alignment_converge_threshold_Pyaw;
}

bool KalmanFilter::zhrGnss(double time, const Vector3d& pned, const Vector3d& vned, const Eigen::Vector3d& pacc, const Vector3d& vacc)
{
    if(!m_ins.odo_pos_buf.query(time, pos_odo))
    {
        return false;
    }
    m_ins.odo_vel_buf.query(time, vel_odo);

    VectorXd z(6,1);
    z.head(3) = pned - pos_odo;
    z.tail(3) = vned - vel_odo;

    MatrixXd H;
    H.resize(6, NUM_KF_STATES);
    H.setZero();
    H(0, KF_STATE_POSITION_X) = 1.0;
    H(1, KF_STATE_POSITION_Y) = 1.0;
    H(2, KF_STATE_POSITION_Z) = 1.0;
    H(3, KF_STATE_VELOCITY_X) = 1.0;
    H(4, KF_STATE_VELOCITY_Y) = 1.0;
    H(5, KF_STATE_VELOCITY_Z) = 1.0;

    MatrixXd R;
    R.resize(6, 6);
    R.setZero();

    R(0, 0) = pacc(0);
    R(1, 1) = pacc(1);
    R(2, 2) = pacc(2);
    R(3, 3) = vacc(0);
    R(4, 4) = vacc(1);
    R(5, 5) = vacc(2);

    if(!update(z, H, R, kv))
    {
        return false;
    }
    return true;
}

bool KalmanFilter::zhrGnssHeading(double time, const Vector3d& vned)
{
    Vector3d euler;
    if(!m_ins.odo_euler_buf.query(time, euler))
    {
        return false;
    }

    if(last_wheel_speed > 2)
    {
        gnss_heading = atan2(vned(1), vned(0));
        gnss_heading_valid = true;
    }
    else if(last_wheel_speed < -2)
    {
        gnss_heading = atan2(vned(1), vned(0));
        gnss_heading = unwrap_diff(gnss_heading+PI, 0);
        gnss_heading_valid = true;
    }
    else
    {
        gnss_heading_valid = false;
        return false;
    }

    VectorXd z;
    z.resize(1);
    double yaw_observe = unwrap_diff(gnss_heading, 0);
    z(0) = unwrap_diff(yaw_observe, euler(2));
    Matrix<double, 1, NUM_KF_STATES> H;
    H.setZero();
    H(KF_STATE_MISALIGN_Z) = 1.0;
    MatrixXd R;
    R.resize(1,1);
    if(vned.norm() > 6)
    {
        R(0, 0) = 2* DEG2RAD / vned.norm();
    }
    else
    {
        R(0, 0) = 6* DEG2RAD / vned.norm();
    }

    R(0,0) = R(0,0) * R(0,0);
    const double kv = 16;

    if(!update(z, H, R, kv))
    {
        return false;
    }
    return true;
}

void KalmanFilter::processImu(const Imu& tImu)
{
    m_mm.push_imu(tImu);
    MotionMonitorProcess();

    if(kfstatus == UNINITIALIZED) return;
    m_ins.strapdown(tImu);
}

bool KalmanFilter::zhrZupt()
{
    Vector3d delta_pos = pos_latch - m_ins.C_ned2wgs.transpose() * m_ins.pos_wgs;
    Vector3d delta_velo = - m_ins.vel_ned;

    double delta_yaw = unwrap_diff(yaw_latch, quat2rpy(m_ins.Q_body2ned)(2));

    m_ins.bg = m_mm.GetGyroMean();

    VectorXd z;
    z.resize(7);

    z << delta_pos(0), delta_pos(1), delta_pos(2), delta_velo(0), delta_velo(1), delta_velo(2), delta_yaw;

    MatrixXd H;
    H.resize(7, NUM_KF_STATES);
    H.setZero();
    H(0, KF_STATE_POSITION_X) = 1.0;
    H(1, KF_STATE_POSITION_Y) = 1.0;
    H(2, KF_STATE_POSITION_Z) = 1.0;
    H(3, KF_STATE_VELOCITY_X) = 1.0;
    H(4, KF_STATE_VELOCITY_Y) = 1.0;
    H(5, KF_STATE_VELOCITY_Z) = 1.0;
    H(6, KF_STATE_VELOCITY_X) = 1.0;

    MatrixXd R;
    R.resize(7, 7);
    R.setZero();

    R(0, 0) = 1e-4;
    R(1, 1) = 1e-4;
    R(2, 2) = 1e-4;
    R(3, 3) = 1e-6;
    R(4, 4) = 1e-6;
    R(5, 5) = 1e-6;
    R(6, 6) = 1e-6;

    if(!update(z, H, R, kv))
    {
        return false;
    }
    return true;
}

localization KalmanFilter::outputEgoStatesMsg(double t)
{
    localization loc;    
    loc.timestamp = t;
    loc.status = kfstatus;

    loc.posned = m_ins.C_ned2wgs.transpose() * m_ins.pos_wgs;
    loc.velned = m_ins.vel_ned;
    loc.euler = quat2rpy(m_ins.Q_body2ned);
    loc.quat = m_ins.Q_body2ned;
    loc.ang_rate = m_ins.ang_rate;
    loc.acceleration = m_ins.acc_body;

    loc.pos_cov = m_kf.P.block<3,3>(0,0);
    loc.vel_cov = m_kf.P.block<3,3>(3,3);
    loc.euler_cov = m_kf.P.block<3,3>(6,6);

    return loc;
    
}