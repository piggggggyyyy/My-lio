#pragma once
#include "ieskf_slam/type/point.h"
#include "ieskf_slam/type/timestamp.h"
#include "ieskf_slam/type/ivox3d/ivox3d.h"
namespace IESKFSlam {
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, Point>;
#endif
    using IVoxPtr = std::shared_ptr<IVoxType>;
    struct PointCloud {
        using Ptr = std::shared_ptr<PointCloud>;  //指针别名
        TimeStamp time_stamp;                     //时间戳
        PCLPointCloudPtr cloud_ptr;               // pcl
        PointCloud() { cloud_ptr = pcl::make_shared<PCLPointCloud>(); }
    };
    extern std::vector<std::vector<Point, Eigen::aligned_allocator<Point>>> nearest_points;
    
}  // namespace IESKFSlam