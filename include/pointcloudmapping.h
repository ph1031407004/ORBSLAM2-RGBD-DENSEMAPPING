#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

//滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//保存点云的库文件
#include <pcl/io/pcd_io.h> 
//添加自定义点云类
#include "PointCloude.h"

//添加关键帧更新相关
#include <condition_variable>
namespace ORB_SLAM2
{
    class PointCloudMapping
    {
        public:
        //定义类型别名
        typedef pcl::PointXYZRGBA PointT;
        typedef pcl::PointCloud<PointT> PointCloud;

        PointCloudMapping(double resolution_,double meank_,double thresh_);

        void InsertKeyFrame(KeyFrame* kf, cv::Mat& color_img, cv::Mat& depth_img,int idk,vector<KeyFrame*> vpKFs);

        PointCloud::Ptr GeneratePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

        void UpdateCloud();

        void Viewer();

        void SaveDenseMap();

        void ShutDown();
        
        
        PointCloud::Ptr mpGlobalCloud; //全局点云变量
        shared_ptr<thread> mptViewerThread; //***这里用处
        vector<KeyFrame*> mvKeyFrames; //关键帧向量数组
        size_t mnLastKeyFrameId = 0;
        mutex mmKeyFrameMutex; //锁 这里还不是很懂

        double resolution = 0.01;//体素栅格大小4cm
        double meank = 50;//*相邻点
        double thresh = 2;//阈值

        //定义相关点云滤波器
        pcl::VoxelGrid<PointT>  voxel;
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;

        //定义关闭显示的相关变量
        bool    shutDownFlag    =false;
        mutex   shutDownMutex;

        //更新全局稠密点云相关定义
        int loopcount = 0; //全局ba之后更新一次 更新后次数加1
        bool loopbusy;      //在显示中防止这边更新的冲突 更新完就赋值为false
        bool cloudbusy;      //点云生成是否繁忙
        vector<KeyFrame*> currentvpKFs; //当前关键帧

        //保存 点云的 数组
        vector<PointCloude>     pointcloud;

        //关键帧更新锁相关
        condition_variable  keyFrameUpdated;
        mutex               keyFrameUpdateMutex;

        //停止点云建图的标志位
        // bool loopbusy;  
        bool bStop = false;
    };
}

#endif