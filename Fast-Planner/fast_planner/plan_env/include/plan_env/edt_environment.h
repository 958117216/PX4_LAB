#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_env/local_map_gyl.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment {
private:
  /* data */
  ObjPrediction obj_prediction_;
  ObjScale obj_scale_;
  double resolution_inv_;
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  EDTEnvironment(/* args */) {
  }
  ~EDTEnvironment() {
  }

  SDFMap::Ptr sdf_map_;
  LocalSDFMap::Ptr localsdf_map_;

  void init();
  void setMap(LocalSDFMap::Ptr map);
  void setObjPrediction(ObjPrediction prediction);
  void setObjScale(ObjScale scale);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  pair<double, Eigen::Vector3d> interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff,
                                                     double& value, Eigen::Vector3d& grad);
  pair<double, Eigen::Vector3d> evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,
                                                    double& dist, Eigen::Vector3d& grad);
  double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& end) {
    localsdf_map_->getRegion(ori, end);
  }

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace fast_planner

#endif