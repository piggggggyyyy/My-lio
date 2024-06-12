#include "ieskf_slam/modules/map/rect_map_manager.h"

#include "ieskf_slam/math/math.h"
#include "pcl/common/transforms.h"
namespace IESKFSlam {
    RectMapManager::RectMapManager(const std::string &config_file_path, const std::string &prefix)
        : ModuleBase(config_file_path, prefix, "RectMapManager") {
        local_map_ptr = pcl::make_shared<PCLPointCloud>();
        kdtree_ptr = pcl::make_shared<KDTree>();
        ivox_ = std::make_shared<IVoxType>(ivox_options_);
        int ivox_nearby_type;
        readParam<float>("map_side_length_2",map_side_length_2,500);
        readParam<float>("map_resolution",map_resolution,0.5);
        readParam<float>("ivox_grid_resolution",ivox_options_.resolution_, 0.2);
        readParam<int>("ivox_nearby_type", ivox_nearby_type, 18);
        if (ivox_nearby_type == 0) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
        } else if (ivox_nearby_type == 6) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
        } else if (ivox_nearby_type == 18) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        } else if (ivox_nearby_type == 26) {
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
        } else {
            //LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
        }
    }

    RectMapManager::~RectMapManager() {}
    void RectMapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q,
                                 const Eigen::Vector3d &pos_t) {

         // /* transform to world frame */
        //  /* decide if need add to map */
        PCLPointCloud scan;
        pcl::transformPointCloud(*curr_scan, scan, compositeTransform(att_q, pos_t).cast<float>());
        if (local_map_ptr->empty()) {
            *local_map_ptr = scan;
            ivox_->AddPoints(scan.points);
        } else {
            
            std::vector<Point,Eigen::aligned_allocator<Point>> points_to_add,points_no_need_downsample;
            int cur_pts = scan.size();
            points_to_add.reserve(cur_pts);
            points_no_need_downsample.reserve(cur_pts);
            std::vector<size_t> index(cur_pts);
            for(size_t i = 0; i < cur_pts; i++){
                index[i] = i;
            }

            std::for_each(std::execution::unseq, index.begin(),index.end(), [&](const size_t &i){
                Point point_world = scan.points[i];
                if(!nearest_points[i].empty()){
                    const std::vector<Point,Eigen::aligned_allocator<Point>> &points_near = nearest_points[i];
                    Eigen::Vector3f center =  ((point_world.getVector3fMap() / map_resolution).array().floor() + 0.5) * map_resolution;
                    Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;
                    if (fabs(dis_2_center.x()) > 0.5 * map_resolution &&
                        fabs(dis_2_center.y()) > 0.5 * map_resolution &&
                        fabs(dis_2_center.z()) > 0.5 * map_resolution) {
                            points_no_need_downsample.emplace_back(point_world);
                            return;
                        }
                    bool need_add = true;
                    float dist = calc_dist(point_world.getVector3fMap(), center);
                    //std::cout << dist << std::endl;
                    if (points_near.size() >= 5) {
                        for (int readd_i = 0; readd_i < 5; readd_i++) {
                            if (calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                                 need_add = false;
                                break;
                            }
                        }
                    }
                    if (need_add) {
                        points_to_add.emplace_back(point_world);
                    }
                }else{
                    points_to_add.emplace_back(point_world);
                }
            });
            //std::cout << "points_to_add:   " << points_to_add.size() << "    points_no_need_downsample  :"  << points_no_need_downsample.size()<<std::endl;
            //ivox_->AddPoints(points_to_add);
            //ivox_->AddPoints(points_no_need_downsample); 
        }
    //         for (auto &&point : scan) {
    //             //no need to do nearest,need to chanbe it to voxel
    //             std::vector<int> ind;
    //             std::vector<float> distance;
    //             kdtree_ptr->nearestKSearch(point, 5, ind, distance);
    //             if (distance[0] > map_resolution) {
    //                 local_map_ptr->push_back(point);
    //             }
    //         }
    //         int left = 0, right = local_map_ptr->size() - 1;
    //         while (left < right) {
    //             while (left < right && abs(local_map_ptr->points[right].x - pos_t.x()) > map_side_length_2 ||
    //                    abs(local_map_ptr->points[right].y - pos_t.y()) > map_side_length_2 ||
    //                    abs(local_map_ptr->points[right].z - pos_t.z()) > map_side_length_2)
    //                 right--;
    //             while (left < right && abs(local_map_ptr->points[left].x - pos_t.x()) < map_side_length_2 &&
    //                    abs(local_map_ptr->points[left].y - pos_t.y()) < map_side_length_2 &&
    //                    abs(local_map_ptr->points[left].z - pos_t.z()) < map_side_length_2)
    //                 left++;
    //             std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
    //         }
    //         local_map_ptr->resize(right + 1);
    //     }

    //     ivox_->AddPoints(local_map_ptr->points);
    //    kdtree_ptr->setInputCloud(local_map_ptr);
       
    }
    void RectMapManager::reset() { 
        
        local_map_ptr->clear(); }
    PCLPointCloudConstPtr RectMapManager::getLocalMap() { return local_map_ptr; }
    KDTreeConstPtr RectMapManager::readKDtree() { return kdtree_ptr; }
    IVoxPtr RectMapManager::readIvox(){
        std::cout << "rect: "<< ivox_ << std::endl;
        return ivox_;}
}  // namespace IESKFSlam