#include "depth2pointcloud/D2PCL.h"

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "D2PCL_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  D2PCL* D2PCLer = new D2PCL(nh, nh_private);

  ros::spin();
  return 0;
}
