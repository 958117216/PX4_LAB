#include "depth2pointcloud/D2PCL.h"

using namespace Eigen;
using namespace std;

D2PCL::D2PCL(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
 :nh_(nh),nh_private_(nh_private)
{
  nh_private_.param("fx", fx_, 554.3826904296875);//相机的标定系数
  nh_private_.param("fy", fy_, 554.3826904296875);
  nh_private_.param("cx", cx_, 320.0);
  nh_private_.param("cy", cy_, 240.0); 

 
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/realsense_plugin/camera/depth/image_raw", 50));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/mavros/local_position/odom", 100));
  sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
  sync_image_odom_->registerCallback(boost::bind(&D2PCL::depthOdomCallback, this, _1, _2));

 pcl_pub_= nh_.advertise<sensor_msgs::PointCloud2>("/depth_cloud", 10);

  occ_timer_ = nh_.createTimer(ros::Duration(0.05), &D2PCL::updatePCLCallback, this);
  vis_timer_ = nh_.createTimer(ros::Duration(0.05), &D2PCL::visCallback, this);

  proj_points_.resize(640 * 480 );//640*480/2/2
  proj_points_cnt = 0;
}

void D2PCL::GetPCL() 
{
  proj_points_cnt = 0;
  uint16_t* row_ptr;
  int cols = depth_image_.cols;
  int rows = depth_image_.rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix(); //四元数转换为旋转矩阵

    for (int v = 0; v < rows; v++) 
    {
      row_ptr = depth_image_.ptr<uint16_t>(v);
      for (int u = 0; u < cols; u++) 
      {
        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / 1000.0;
        if(depth == 0) continue;
        proj_pt(0) = (u - cx_) * depth / fx_;
        proj_pt(1) = (v - cy_) * depth / fy_;
        proj_pt(2) = depth;

          Eigen::Vector3d pt_base;
          pt_base(0)= proj_pt(2);
          pt_base(1)= -proj_pt(0);
          pt_base(2)= -proj_pt(1);
           
           proj_pt=pt_base;
         //proj_pt = camera_r * pt_base + camera_pos_;

        //if (u == 240 && v == 320) std::cout << "depth: " << depth << std::endl;
        proj_points_[proj_points_cnt++] = proj_pt;
      }
    }

}

void D2PCL::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) 
{
  /* get pose */
  camera_pos_(0) = odom->pose.pose.position.x;
  camera_pos_(1) = odom->pose.pose.position.y;
  camera_pos_(2) = odom->pose.pose.position.z;
  camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  cv_ptr->image.copyTo(depth_image_);

  PCL_need_update_ = true;
}

void D2PCL::publishDepth() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < proj_points_cnt; ++i)
   {
    pt.x = proj_points_[i][0];
    pt.y = proj_points_[i][1];
    pt.z = proj_points_[i][2];
    cloud.push_back(pt);
   }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id ="base_link";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  cloud_msg.header.stamp =ros::Time::now();
  pcl_pub_.publish(cloud_msg);
}

void D2PCL::visCallback(const ros::TimerEvent& /*event*/)
{
   publishDepth();
}

void D2PCL::updatePCLCallback(const ros::TimerEvent& /*event*/)
 {
  if (!PCL_need_update_) return;

  /* update occupancy */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  GetPCL();


  t2 = ros::Time::now();





  PCL_need_update_ = false;

}