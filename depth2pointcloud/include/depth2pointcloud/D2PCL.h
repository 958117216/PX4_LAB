#ifndef _D2PCL_H_
#define _D2PCL_H_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace Eigen;

class D2PCL
{
    public:

   cv::Mat depth_image_;
   Eigen::Quaterniond camera_q_;
   Eigen::Vector3d camera_pos_;
   double cx_,cy_,fx_,fy_;
   std::vector<Eigen::Vector3d> proj_points_;
   int proj_points_cnt;
   bool PCL_need_update_;
  
    D2PCL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~D2PCL(){}

    void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
    void GetPCL() ;
     void publishDepth();
    void updatePCLCallback(const ros::TimerEvent& /*event*/);
    void visCallback(const ros::TimerEvent& /*event*/);

    private:

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  ros::NodeHandle nh_,nh_private_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  SynchronizerImageOdom sync_image_odom_;

 ros::Subscriber indep_depth_sub_, indep_odom_sub_;
 ros::Publisher pcl_pub_;
 ros::Timer occ_timer_, vis_timer_;

};


#endif