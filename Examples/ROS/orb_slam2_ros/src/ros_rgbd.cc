/**
 * @file ros_rgbd.cc
 * @author guoqing (1337841346@qq.com)
 * @brief ORB RGB-D 输入的ROS节点实现
 * @version 0.1
 * @date 2019-08-06
 * 
 * @copyright Copyright (c) 2019
 * 
 */


/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
//添加新头文件
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include"../../../include/Converter.h"

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>


#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
//定义全局数据类型

ros::Publisher pub_camera_pose;
cv::Mat Tcw ;

void pubOdomPose(cv::Mat& T)
{
    cv::Mat Twc = T.inv();
    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3); // Rotation information 旋转
    cv::Mat twc = Twc.rowRange(0,3).col(3); // translation information 平移
    
    Eigen::Matrix<double,3,3> eigMat ;
    eigMat <<Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                    Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                    Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2);
    Eigen::Quaterniond q(eigMat);//得到四元数
    //更简单的方法 参考   但是报错 Converter’ was not declared in this scope
    //Converter converter;
   // std::vector<float> v = converter.toQuaternion(Rwc);
    nav_msgs::Odometry odom_msg;
    
    // std_msgs::Header header;
    // header.stamp = ros::Time::now();
    // odometry.header = header;
    odom_msg.header.frame_id = "vodom";
    odom_msg.header.stamp = ros::Time::now();

    odom_msg.pose.pose.position.x = twc.at<float>(0,0);
    odom_msg.pose.pose.position.y = twc.at<float>(1,0);
    odom_msg.pose.pose.position.z = twc.at<float>(2,0);
      
    //std::vector<float> v = converter.toQuaternion(Rwc);
    odom_msg.pose.pose.orientation.x  = q.x();
    odom_msg.pose.pose.orientation.y  = q.y();
    odom_msg.pose.pose.orientation.z  = q.z();
    odom_msg.pose.pose.orientation.w  = q.w();

    pub_camera_pose.publish(odom_msg);
}
class ImageGrabber
{
public:
    
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");//
    ros::start();//1.初始化 启动相关线程

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);//构建一个类 捕获RGBD图像并进行跟踪

    //ROS下的话题
    /*ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);*/
    
    //订阅话题
    ros::NodeHandle nh("~");
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 1);
    //发布话题
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcloutput", 1);
    pub_camera_pose = nh.advertise<nav_msgs::Odometry>("camera_pose", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    //时间戳对齐
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2)); // 回调函数 这儿的_1和_2是啥意思

    //ros::spin();
     ros::Rate r(10);
    while(ros::ok())
    {
        //获取全局点云信息
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Out_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
        pcl::PointCloud<pcl::PointXYZRGBA> cloud2;//旋转之后的点云
        sensor_msgs::PointCloud2 pub_cloud;

        //Out_cloud=igb.mpSLAM->Getglobalcloud();
        Out_cloud=igb.mpSLAM->Getglobalcloud2();//加上锁之后

        cloud2 = (* Out_cloud);//拷贝

        //点云和之后的栅格地图不对应，这里将点云做一个旋转变换
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(-M_PI*43/64, Eigen::Vector3f(1,0,0)));     //roll pitch yaw 
        //考虑到点云还存在一定平移
        transform.translation()<<-2,-3.2,1.55;
        pcl::transformPointCloud(cloud2, cloud2, transform);

        //点云信息发布
        pcl::toROSMsg(cloud2, pub_cloud);
       // pcl::fromPCLPointCloud2(cloud, output);
        //ROS_INFO_STREAM( "ROS cloud, size =  " << Out_cloud->points.size() );
        pub_cloud.header.stamp=ros::Time::now();
        pub_cloud.header.frame_id  ="world";
        pcl_pub.publish(pub_cloud);
        //位姿信息发布
        if(!Tcw.empty())
            pubOdomPose(Tcw);//publish在函数的结尾
        else
           cout<<"Tcw is empty..."<<endl;
        ros::spinOnce();
        r.sleep();
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();



    return 0;
}
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}


