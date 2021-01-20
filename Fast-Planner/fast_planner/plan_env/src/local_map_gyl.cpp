#include <plan_env/local_map_gyl.h>


void LocalSDFMap::initMap(ros::NodeHandle& nh) {
  node_ = nh;

  /* get parame*/
  node_.param("local_map/resolution", mp_.resolution_, 0.1);//0.1 比例尺 mp_：SDFMap私有成员


  node_.param("local_map/map_origin_x", mp_.map_origin_static(0), -4.0);//-4
  node_.param("local_map/map_origin_y", mp_.map_origin_static(1), -6.0);//-6
  node_.param("local_map/map_origin_z", mp_.map_origin_static(2), -1.0);//-1

  node_.param("local_map/map_origin_x", mp_.map_origin_(0), -4.0);//-4
  node_.param("local_map/map_origin_y", mp_.map_origin_(1), -6.0);//-6
  node_.param("local_map/map_origin_z", mp_.map_origin_(2), -1.0);//-1

  node_.param("local_map/map_size_x", mp_.map_size_(0), 12.0);//12
  node_.param("local_map/map_size_y", mp_.map_size_(1), 12.0);//12
  node_.param("local_map/map_size_z", mp_.map_size_(2), 6.0);//6

  node_.param("local_map/frame_id", mp_.frame_id_, string("world"));
  
  mp_.show_esdf_time_=false;

  mp_.resolution_inv_ = 1 / mp_.resolution_;//10

  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
// 地图的网格数量大小 mp_.map_voxel_num_(0)=120  mp_.map_voxel_num_(1)=120  mp_.map_voxel_num_(2)=60 
  mp_.map_min_boundary_ = mp_.map_origin_;//0 -3  -1
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;//6 3 3

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();//0 0 0
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();//60 60 40

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
//60*60*40
  md_.occupancy_buffer_ = vector<double>(buffer_size,-0.87);//赋值-0.87
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);


  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);

  /* init callback */ //时间同步器 message_filters::sync_policies::ApproximateTime的定义

/*订阅部分，回调函数中更新占据栅格地图*/
   cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/sdf_map/cloud", 100));
   odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/sdf_map/odom", 200));

    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&LocalSDFMap::cloudOdomCallback, this, _1, _2));


/*由占据栅格地图计算得：esdf地图*/
esdf_timer_ = node_.createTimer(ros::Duration(0.05), &LocalSDFMap::updateESDFCallback, this);

/*发布各种地图信息*/
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &LocalSDFMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/local_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/local_map/occupancy_inflate", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/local_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/local_map/update_range", 10);


  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);//噪声
  rand_noise2_ = normal_distribution<double>(0, 0.2);//噪声
  random_device rd;
  eng_ = default_random_engine(rd());
}

void LocalSDFMap::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& img,const nav_msgs::OdometryConstPtr& odom)
 {
      /* get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                         odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  md_.has_odom_=true;
                

  LocalMap_ParametersUpdate();


    /*get cloud*/
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_) {
     std::cout << "no odom!" << std::endl;
    return;
  }

  //std::cout << "cloudOdomCallback  Yes!" << std::endl;

  if (latest_cloud.points.size() == 0) return;

    //判断是不是相机的三轴pos是不是NAN 
  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return; 

  resetBuffer();   //清空 occ和dis两个buff

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = 2;//ceil(mp_.obstacles_inflation_ / mp_.resolution_); // 0.099/0.1=0.99  ——>1
  int inf_step_z = 1;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
   {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

     p3d=position_change( p3d, md_.camera_pos_,md_.camera_q_);

    Eigen::Vector3i inf_pt;

    if (p3d(0) >  mp_.map_min_boundary_(0)&& p3d(0) <mp_.map_max_boundary_(0)&&
         p3d(1) > mp_.map_min_boundary_(1)&& p3d(1) <mp_.map_max_boundary_(1)&&
         p3d(2) > mp_.map_min_boundary_(2)&& p3d(2) <mp_.map_max_boundary_(2))
   {
      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {

            p3d_inf(0) = p3d(0)  + x * mp_.resolution_;
            p3d_inf(1) = p3d(1)  + y * mp_.resolution_;
            p3d_inf(2) = p3d(2)  + z * mp_.resolution_;


           posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;  
          }
    }

  }
  md_.esdf_need_update_ = true;
 }


 void LocalSDFMap::resetBuffer() 
 {

  /* reset occ and dist buffer */

        md_.occupancy_buffer_inflate_=vector<char>(mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2), 0);  
        md_.distance_buffer_= vector<double>(mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2), 10000);

}

void LocalSDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}


template <typename F_get_val, typename F_set_val>
void LocalSDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

          
  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }
  
  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void LocalSDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = mp_.map_min_idx_;
  Eigen::Vector3i max_esdf = mp_.map_max_idx_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();//返回 编译器允许的 double 型数 最大值。
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_); 
      }
}

void LocalSDFMap::visCallback(const ros::TimerEvent& /*event*/) 
{
  publishMap();
 // publishMapInflate(false);
   publishUpdateRange();
   publishESDF();

//  publishUnknown();
  // publishDepth();
}

void LocalSDFMap::publishMap() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int x = 0; x <= mp_.map_max_idx_[0]; ++x)
    for (int y = 0; y <=mp_.map_max_idx_[1]; ++y)
      for (int z = 0; z <= mp_.map_max_idx_[2]; ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
      
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void LocalSDFMap::publishUpdateRange()
 {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;

  indexToPos(mp_.map_min_idx_, esdf_min_pos);
  indexToPos(mp_.map_max_idx_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void LocalSDFMap::publishESDF() 
{
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

 Eigen::Vector3i min_cut = mp_.map_min_idx_;
 Eigen::Vector3i max_cut = mp_.map_max_idx_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = 0.3; //esdf 显示高度

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  //ROS_INFO("pub esdf");
}


double LocalSDFMap::getResolution() { return mp_.resolution_; }

void LocalSDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& end)
{
  ori = mp_.map_origin_, end = mp_.map_size_+mp_.map_origin_;
}

void LocalSDFMap::getOrigin(Eigen::Vector3d& ori)
{
  ori = mp_.map_origin_;
}

 void  LocalSDFMap::LocalMap_ParametersUpdate()
 { 
   mp_.map_origin_=md_.camera_pos_+mp_.map_origin_static;
   //cout<<mp_.map_origin_.transpose()<<endl;
   mp_.map_min_boundary_ = mp_.map_origin_;
   mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
 }

 Eigen::Vector3d LocalSDFMap::position_change(Eigen::Vector3d pc, Eigen::Vector3d tc,Eigen::Quaterniond qc)
 {       

       Eigen::Vector3d pw;

        qc.normalize();//四元数使用之前必须归一化处理
        
       //pw=qc.inverse()*(pc-tc);
          pw=qc*pc+tc;  //在世界坐标系下，相机的位姿对应Twc
          
      return pw;
 }