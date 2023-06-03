
#include "pointcloudmapping.h"
#include "Converter.h"


namespace ORB_SLAM2
{
    PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
    {

        this->resolution = resolution_;
        this->meank = thresh_;
        this->thresh = thresh_;
        //滤波参数设置
        statistical_filter.setMeanK(meank);
        statistical_filter.setStddevMulThresh(thresh);
        voxel.setLeafSize( resolution, resolution, resolution);

        std::cout << "Point cloud mapping has structed. " << std::endl;
        //下面这句代码理解
        mpGlobalCloud = boost::make_shared< PointCloud >( );
        mptViewerThread = make_shared<thread>(bind(&PointCloudMapping::Viewer, this));
    }

    void PointCloudMapping::InsertKeyFrame(KeyFrame* kf, cv::Mat &color_img, cv::Mat &depth_img,int idk,vector<KeyFrame*> vpKFs)
    {
        cout << "recieve a keyframe, id = " << idk << "第"<< kf->mnId << endl;//

        unique_lock<mutex> lck(mmKeyFrameMutex);
        mvKeyFrames.push_back(kf);
        currentvpKFs = vpKFs;

        //实例化点云
        PointCloude pointcloude;
        pointcloude.pcID = idk;
        //pointcloude.T = ORB_SLAM2::Converter::toMatrix4d( kf->GetPose() );
        pointcloude.T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
         //cout<<"size T:"<<pointcloude.T.matrix()<<endl;
        pointcloude.pcE = GeneratePointCloud(kf,color_img,depth_img);
        pointcloud.push_back(pointcloude);
        cout <<"test size:"<< pointcloud.size()<<"idk:"<<idk<<endl;
        //GeneratePointCloud(kf, color_img, depth_img);
        keyFrameUpdated.notify_one();
    }

     pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::GeneratePointCloud(KeyFrame* kf, cv::Mat &color_img, cv::Mat &depth_img)
    {

        // cv::imshow("color img", color_img);
        // cv::imshow("depth img", depth_img);
        PointCloud::Ptr tmp (new PointCloud());
        for ( int m=0; m<depth_img.rows; m+=3 )
        {
            for ( int n=0; n<depth_img.cols; n+=3 )
            {
                float d = depth_img.ptr<float>(m)[n];
                if (d < 0.01 || d>5)
                    continue;
                PointT p;
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;
                
                p.b = color_img.ptr<uchar>(m)[n*3];
                p.g = color_img.ptr<uchar>(m)[n*3+1];
                p.r = color_img.ptr<uchar>(m)[n*3+2];

                // cout << p.x << " " << p.y << " " << p.z << endl;
                    
                tmp->points.push_back(p);
            }
        }
        cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<tmp->points.size()<<endl;
        //cout << "The keyframe has point clouds: " << tmp->points.size() << endl;
        kf->mptrPointCloud = tmp;//传递点云指针
        return tmp;
    }

    void PointCloudMapping::UpdateCloud()
    {
            if(!cloudbusy)
        {
            loopbusy = true;
            cout<<"startloopmappoint"<<endl;
            PointCloud::Ptr tmp1(new PointCloud);
            for (int i=0;i<currentvpKFs.size();i++)
            {
                for (int j=0;j<pointcloud.size();j++)
                {   
                    if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId) //这里的判断是不是有问题 之前的IDX更新存在问题，这里还可以考虑提出坏的当前关键帧
                    {   
                        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
                        //更新的全局不对，考虑这里数据格式转换的问题 转换没问题，验证过了
                       // cout << "toSE3Quat T " << T << endl; 
                        PointCloud::Ptr cloud(new PointCloud);
                        pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
                        //pcl::transformPointCloud(*pointcloud[j].pcE, *cloud,Converter::toMatrix4d(currentvpKFs[i]->GetPoseInverse()) );
                        *tmp1 +=*cloud;

                        //cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
                        continue;
                        
                    }
                }
            }
            cout<<"finishloopmap"<<endl;
            //滤波
            /*
            PointCloud::Ptr tmp2(new PointCloud());
            voxel.setInputCloud( tmp1 );
            voxel.filter( *tmp2 );
            mpGlobalCloud->swap( *tmp2 );*/
             
            //viewer.showCloud( globalMap );

            mpGlobalCloud->swap( *tmp1 );//先不更新
            //把更新的保存下来  看看问题
            pcl::io::savePCDFile( "result_ba.pcd", *tmp1);
            cout<<"ba save finished"<<endl;
            loopbusy = false;
            //cloudbusy = true;
            loopcount++;//全局BA一次更新一次全局点云
        
        }
    }
    void PointCloudMapping::SaveDenseMap()
    {
        pcl::io::savePCDFile( "result.pcd", *mpGlobalCloud);
        cout<<"GlobalMap save finished"<<endl;
    }
    void PointCloudMapping::ShutDown()//在system.cc中调用
    {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();//**
        }
        mptViewerThread->join();
    }
    void PointCloudMapping::Viewer()
    {
        pcl::visualization::CloudViewer viewer("Dense map viewer");
        while(1)
        {
            {
                unique_lock<mutex> lck_shutdown( shutDownMutex );
                if (shutDownFlag)
                {
                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
                keyFrameUpdated.wait( lck_keyframeUpdated );//等待关键帧插入后继续执行 /shutdown之后执行
            }   
            if(loopbusy || bStop)//bStop在哪里调用的？
            {
            //cout<<"loopbusy || bStop"<<endl;
                //continue;
            }
            size_t N = 0;
            {
                unique_lock<mutex> lck(mmKeyFrameMutex);
                N = mvKeyFrames.size();
            }

            if(N == mnLastKeyFrameId)
            {
                cloudbusy=false;//点云更新可以进行
            }
            cloudbusy=true;

                
            for(size_t i = mnLastKeyFrameId; i < N; i++)
            {
                PointCloud::Ptr p (new PointCloud);
                pcl::transformPointCloud(*(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
                //cout<<"处理好第i个点云"<<i<<endl;
                *mpGlobalCloud += *p;
            }
            //这下面的作用
            mnLastKeyFrameId = N;
            cloudbusy = false;
            cout << "Total has point clouds: " << mpGlobalCloud->points.size() << endl;
               
            
            /*//滤波  加上这部分代码就会段错误
            PointCloud::Ptr tmp1 ( new PointCloud );       
            statistical_filter.setInputCloud(mpGlobalCloud);
            statistical_filter.filter( *tmp1 );
            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud( tmp1 );
            voxel.filter( *mpGlobalCloud );
            //globalMap->swap( *tmp );*/
            


            viewer.showCloud(mpGlobalCloud);

            cout<<"show global map, size="<<N<<"   "<<mpGlobalCloud->points.size()<<endl;
            
        }
    }
}
