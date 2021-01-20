
#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  px4_state_flag_=1;
  exec_state_  = FSM_EXEC_STATE::INIT;
  trigger_     = false;
  have_target_ = false;
  have_odom_   = false;
  offboard_= false;
   
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    pos_setpoint.position.z = 1.0;

    pos_setpoint.yaw = 0;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);//1 手动选择目标 munual target
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);//1.5
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);//2

  nh.param("fsm/waypoint_num", waypoint_num_, -1);  //2
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);//指针重新指向（赋值）
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);//定时0.01s
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);//定时0.05s
  //state_timer_= nh.createTimer(ros::Duration(0.1), &KinoReplanFSM::monitorStateCallback, this);//定时0.1s 设置失败 原因不详
 /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
// 订阅指定主题，并指定回调函数，1000为队列大小，当我们来不及处理消息时，会存储在该队列中，若队列元素大于1000，则会抛弃老的消息
  waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  state_sub_ = nh.subscribe("/mavros/state", 10,&KinoReplanFSM::stateCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);
  imu_sub_=  nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &KinoReplanFSM::imuCallback,this);
  /*
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  // 该句告诉master主控节点，我们将在chatter主题中发布std_msgs的String消息，在我们发布消息时，
  // 主控节点将会告知所有订阅该主题的节点，消息队列大小为1000，即在队列里有消息超过1000个之后，才会丢弃以前老的消息
  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  setpoint_raw_local_pub=nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  //nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);//防止px4退出offboard模式

    /*service 失败原因不详*/
    //  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    // set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) 
{
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  if(px4_state_flag_==0)
  {
     //px4_state_flag_=1;//获得目标点，更改px4_state_flag_进入offboard模式
      cout << " px4_state_flag_=1" << endl;
  }

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
  {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
  } 
  
  else if (target_type_ == TARGET_TYPE::PRESET_TARGET) 
  {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

 void KinoReplanFSM::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
           Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            //Transform the Quaternion to euler Angles
            // Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
            
            // std::cout<<"euler:"<<euler_fcu.transpose()<<std::endl;

           imu_acc_(0)=msg->linear_acceleration.x;
           imu_acc_(1)=msg->linear_acceleration.y;
          imu_acc_(2)=msg->linear_acceleration.z;

 }
void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e)
 {  

        if(current_state.mode != "OFFBOARD" || !trigger_)
        {  
              geometry_msgs::PoseStamped pose;
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 1;

              setpoint_raw_local_pub.publish(pose); //触发前 悬停
        }


  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
   {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    if (!offboard_) cout << "no  OFFBOARD." << endl;
    fsm_num = 0;
  }

  switch(exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      if (!offboard_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);//起始于(0，0)，提取块大小为(3，1)	
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* &&(end_pt_ - pos).norm() < 0.5 */
      if ( t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
         //cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) { //1.5
        //cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
     
      // start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      // start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      // start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      // start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      // start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];
      

       start_pt_  = (info->position_traj_.evaluateDeBoorT(t_cur)-odom_pos_)*0.1+odom_pos_;
       start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool  new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

            /* publish traj */
        /*消息格式
        int32 order      //阶数
        int64 traj_id    //id
        time start_time  //开始路径规划时间

        float64[] knots   //节点
        geometry_msgs/Point[] pos_pts    //轨迹控制点

        float64[] yaw_pts  //偏航角 控制点
        float64 yaw_dt     //间隔时间
        */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), false, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}


void KinoReplanFSM::stateCallback(const mavros_msgs::StateConstPtr& msg) //订阅当前px4的状态
{
   current_state = *msg;

  if( current_state.mode == "OFFBOARD" )         offboard_=true;
}

void   KinoReplanFSM::monitorStateCallback(const ros::TimerEvent& e)
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    
   ros::Time last_request = ros::Time::now();


    if(!current_state.connected)   return;
    

    if(px4_state_flag_==0) //无触发 不用更改无人机的状态
    {

    }
    else if(px4_state_flag_==1)//已有目标点 更改无人机状态为OFFBOARD模式
    {

      if( current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(2.0)))
         {
                  if( set_mode_client_.call(offb_set_mode) &&offb_set_mode.response.mode_sent)
                  {
                      ROS_INFO("Offboard enabled");
                  }
                  ROS_INFO("Offboard quest!");
                  last_request = ros::Time::now();
          } else 
              {
                  if( !current_state.armed &&(ros::Time::now() - last_request > ros::Duration(2.0)))
                  {
                      if( arming_client_.call(arm_cmd) &&arm_cmd.response.success)
                      {
                          ROS_INFO("Vehicle armed");
                      }
                      last_request = ros::Time::now();
                           ROS_INFO("armed  quest!");
                  }
              }
    }else if(px4_state_flag_==2)
    {



    }




}



// KinoReplanFSM::
}  // namespace fast_planner
