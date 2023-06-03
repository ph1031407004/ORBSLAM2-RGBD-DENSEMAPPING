#ifndef POINTCLOUDE_H
#define POINTCLOUDE_H
#include "pointcloudmapping.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/core/core.hpp>
#include <mutex>

namespace ORB_SLAM2
{

class PointCloude
{
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
public:
    //主要存储了下面三个信息 点云指针，空间的3D位姿，以及对应帧的id
    PointCloud::Ptr pcE;//点云指针
    Eigen::Isometry3d T;//位姿 这里转换矩阵T本质就是4*4的矩阵类型
    int pcID; //id
//protected:    
};
} //namespace ORB_SLAM
#endif // POINTCLOUDE_H