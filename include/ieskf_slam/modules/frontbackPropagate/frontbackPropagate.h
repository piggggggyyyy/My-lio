#pragma once
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/modules/invkf/invkf.h"
#include "ieskf_slam/type/measure_group.h"
namespace IESKFSlam {
    class FrontbackPropagate {
    private:
        struct IMUPose6d {
            double time;
            Eigen::Vector3d acc;
            Eigen::Vector3d angvel;
            Eigen::Vector3d vel;
            Eigen::Vector3d pos;
            Eigen::Quaterniond rot;
            IMUPose6d(double time_ = 0, Eigen::Vector3d a_ = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d av_ = Eigen::Vector3d::Zero(), Eigen::Vector3d v_ = Eigen::Vector3d::Zero(),
                      Eigen::Vector3d p_ = Eigen::Vector3d::Zero(),
                      Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity()) {
                time = time_;
                acc = a_;
                angvel = av_;
                vel = v_;
                pos = p_;
                rot = q_;
            }
        };
        Eigen::Vector3d acc_s_last;
        Eigen::Vector3d angvel_last;

    public:
        double imu_scale;
        IMU last_imu_;
        double last_lidar_end_time_;
        IMU last_imu;
        FrontbackPropagate();
        ~FrontbackPropagate();
        void propagate(MeasureGroup &mg, IESKF::Ptr ieskf_ptr);
        void propagate(MeasureGroup &mg, INVKF::Ptr invkf_ptr);
    };
}  // namespace IESKFSlam