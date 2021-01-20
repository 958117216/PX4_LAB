#ifndef _LOCAL_MAP_GYL_H
#define _LOCAL_MAP_GYL_H

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
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/sdf_map.h>

using namespace std;



class LocalSDFMap {
public:
  LocalSDFMap() {}
  ~LocalSDFMap() {}

  enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

  // occupancy map management 占用地图管理
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  void LocalMap_ParametersUpdate();
  Eigen::Vector3d position_change(Eigen::Vector3d pc, Eigen::Vector3d tc,Eigen::Quaterniond qc);

  inline void  posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) ;
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  inline void boundIndex(Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  // distance field management
  inline double getDistance(const Eigen::Vector3d& pos);
  inline double getDistance(const Eigen::Vector3i& id);
  inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
  // /inline void setLocalRange(Eigen::Vector3d min_pos, Eigen::Vector3d
  // max_pos);

  void updateESDF3d();
  void getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                    vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad,
                    int sign = 1);  // 1 pos, 2 neg, 3 combined
  void initMap(ros::NodeHandle& nh);

  void publishMap();
  void publishMapInflate(bool all_info = false);
  void publishESDF();
  void publishUpdateRange();

  void publishUnknown();
  void publishDepth();

  void checkDist();
  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  double getResolution();
  void getOrigin(Eigen::Vector3d& ori);
  int getVoxelNum();


  typedef std::shared_ptr<LocalSDFMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  // get depth image and camera pose


  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& img,const nav_msgs::OdometryConstPtr& odom);

  void depthCallback(const sensor_msgs::ImageConstPtr& img);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);

  // update occupancy by raycasting, and update ESDF
  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  // main update process
  void clearAndInflateLocalMap();

  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyCloudOdom;

  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;
  SynchronizerCloudOdom sync_cloud_odom_;

  ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
  ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
  ros::Publisher unknown_pub_, depth_pub_;
  ros::Timer occ_timer_, esdf_timer_, vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function============================== */
inline void LocalSDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void LocalSDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline bool LocalSDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

inline bool LocalSDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline int LocalSDFMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int LocalSDFMap::toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void LocalSDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline double LocalSDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

inline double LocalSDFMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

#endif
