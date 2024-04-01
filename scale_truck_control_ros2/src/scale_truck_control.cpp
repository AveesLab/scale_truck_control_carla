#include "scale_truck_control/ScaleTruckController.hpp"

using namespace std::chrono_literals;

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController()
  : Node("scale_truck_control_node", rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))   
{
  if (!readParameters()) {
    rclcpp::shutdown();  
  }

  //RCLCPP_INFO(this->get_logger(), "waitKeyDelay1 : %d\n", waitKeyDelay_);
  init();
}

ScaleTruckController::~ScaleTruckController() {
  isNodeRunning_ = false;

  ros2_msg::msg::Xav2lrc msg;
  msg.tar_vel = ResultVel_;
  {
    std::scoped_lock lock(dist_mutex_);
    msg.steer_angle = AngleDegree_;
    msg.cur_dist = distance_;
  }
  {
    std::scoped_lock lock(rep_mutex_);
    msg.tar_dist = TargetDist_;
    msg.emergency_flag = Emergency_; 
  }

  LrcPublisher_->publish(msg);
  controlThread_.join();
  tcpThread_.join();

  delete cmd_data_;

  RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] Stop.");
  //printf("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  /***********/
  /* LV Index*/
  /***********/
  this->get_parameter_or("params/index", index_, 0);

  /*******************/
  /* Velocity Option */
  /*******************/
  this->get_parameter_or("params/target_vel", TargetVel_, 0.0f); // km/h

  /*******************/
  /* Distance Option */
  /*******************/
  this->get_parameter_or("params/target_dist", TargetDist_, 14.0f); // m
  this->get_parameter_or("params/lv_stop_dist", LVstopDist_, 3.0f); // m
  this->get_parameter_or("params/fv_stop_dist", FVstopDist_, 3.0f); // m

  /***************/
  /* View Option */
  /***************/
  this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, true);

  return true;
}

void ScaleTruckController::init() 
{
  RCLCPP_INFO(this->get_logger(), "[ScaleTruckController] init()");
  gettimeofday(&init_, NULL);

  std::string LaneTopicName;
  int LaneQueueSize;
  std::string RearTopicName;
  int RearQueueSize;
  std::string objectTopicName;
  int objectQueueSize;
  std::string LrcSubTopicName;
  int LrcSubQueueSize;
  std::string CmdSubTopicName;
  int CmdSubQueueSize;

  std::string LrcPubTopicName;
  int LrcPubQueueSize;
  std::string CmdPubTopicName;
  int CmdPubQueueSize;
  std::string LanePubTopicName;
  int LanePubQueueSize;


  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/lane_to_xavier/topic", LaneTopicName, std::string("lane2xav_msg"));
  this->get_parameter_or("subscribers/lane_to_xavier/queue_size", LaneQueueSize, 1);
  this->get_parameter_or("subscribers/rearlane_to_xavier/topic", RearTopicName, std::string("rear2xav_msg"));
  this->get_parameter_or("subscribers/rearlane_to_xavier/queue_size", RearQueueSize, 1);
  this->get_parameter_or("subscribers/obstacle_reading/topic", objectTopicName, std::string("min_distance"));
  this->get_parameter_or("subscribers/obstacle_reading/queue_size", objectQueueSize, 10);
  this->get_parameter_or("subscribers/lrc_to_xavier/topic", LrcSubTopicName, std::string("lrc2xav_msg"));
  this->get_parameter_or("subscribers/lrc_to_xavier/queue_size", LrcSubQueueSize, 1);
  this->get_parameter_or("subscribers/cmd_to_xavier/topic", CmdSubTopicName, std::string("/cmd2xav_msg"));
  this->get_parameter_or("subscribers/cmd_to_xavier/queue_size", CmdSubQueueSize, 10);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/xavier_to_lrc/topic", LrcPubTopicName, std::string("xav2lrc_msg"));
  this->get_parameter_or("publishers/xavier_to_lrc/queue_size", LrcPubQueueSize, 1);
  this->get_parameter_or("publishers/xavier_to_cmd/topic", CmdPubTopicName, std::string("xav2cmd_msg"));
  this->get_parameter_or("publishers/xavier_to_cmd/queue_size", CmdPubQueueSize, 10);
  this->get_parameter_or("publishers/xavier_to_lane/topic", LanePubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("publishers/xavier_to_lane/queue_size", LanePubQueueSize, 1);

  /******************/
  /* Ros Qos Option */
  /******************/
  rclcpp::QoS CmdSubQos(CmdSubQueueSize);
  CmdSubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  CmdSubQos.durability(rclcpp::DurabilityPolicy::Volatile);

  rclcpp::QoS CmdPubQos(CmdPubQueueSize);
  CmdPubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  CmdPubQos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  LrcSubscriber_ = this->create_subscription<ros2_msg::msg::Lrc2xav>(LrcSubTopicName, LrcSubQueueSize, std::bind(&ScaleTruckController::LrcSubCallback, this, std::placeholders::_1));

  CmdSubscriber_ = this->create_subscription<ros2_msg::msg::Cmd2xav>(CmdSubTopicName, CmdSubQos, std::bind(&ScaleTruckController::CmdSubCallback, this, std::placeholders::_1));

  LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(LaneTopicName, LaneQueueSize, std::bind(&ScaleTruckController::LaneSubCallback, this, std::placeholders::_1));

  DistSubscriber_ = this->create_subscription<ros2_msg::msg::Obj2xav>(objectTopicName, objectQueueSize, std::bind(&ScaleTruckController::DistCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  LrcPublisher_ = this->create_publisher<ros2_msg::msg::Xav2lrc>(LrcPubTopicName, LrcPubQueueSize);  
  CmdPublisher_ = this->create_publisher<ros2_msg::msg::Xav2cmd>(CmdPubTopicName, CmdPubQos);  
  LanePublisher_=this->create_publisher<ros2_msg::msg::Xav2lane>(LanePubTopicName,LanePubQueueSize); 

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;

  lane_coef_.coef.resize(3);

  e_values_.resize(3);

  /************/
  /* CMD Data */
  /************/
  cmd_data_ = new ros2_msg::msg::Xav2cmd;
  cmd_data_->src_index = index_;
  cmd_data_->tar_index = 20;  //Control center

  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  controlThread_ = std::thread(&ScaleTruckController::spin, this);
  tcpThread_ = std::thread(&ScaleTruckController::reply, this, cmd_data_); //send to PC_Commend 
}

/***************/
/* cmd publish */
/***************/
void ScaleTruckController::reply(ros2_msg::msg::Xav2cmd* cmd)
{
  while(isNodeRunning_)
  {
    {
      std::scoped_lock lock(lane_mutex_, rlane_mutex_, vel_mutex_, dist_mutex_);
      cmd->tar_vel = TargetVel_;
      cmd->cur_vel = CurVel_;
      cmd->cur_dist = distance_;
      cmd->cur_angle = AngleDegree_;
      cmd->req_flag = req_flag_;

      cmd->coef.resize(3);
      cmd->coef[0].a = lane_coef_.coef[0].a;
      cmd->coef[0].b = lane_coef_.coef[0].b;
      cmd->coef[0].c = lane_coef_.coef[0].c;
      cmd->coef[1].a = lane_coef_.coef[1].a;
      cmd->coef[1].b = lane_coef_.coef[1].b;
      cmd->coef[1].c = lane_coef_.coef[1].c;
      cmd->coef[2].a = lane_coef_.coef[2].a;
      cmd->coef[2].b = lane_coef_.coef[2].b;
      cmd->coef[2].c = lane_coef_.coef[2].c;

    }

    CmdPublisher_->publish(*cmd);

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

float prev_dist = 0;
void ScaleTruckController::objectdetectInThread() 
{
  float dist, dist_tmp;
  dist_tmp = 100.1f;
  ros2_msg::msg::Xav2lane Lane_;
  /**************/
  /* Lidar Data */
  /**************/
  {
    std::scoped_lock lock(object_mutex_);         
    if(dist_tmp >= mindist_)
    {
      if(prev_dist != 0 ) {
        float dist_gap = mindist_ - prev_dist;
        if (dist_gap < 3.0f && dist_gap >-3.0f) {
          dist_tmp = mindist_;
          prev_dist = mindist_;
        }
        else {
          dist_tmp = mindist_;
           prev_dist = mindist_;
        }
      }
      else {
	if (mindist_ > 13.5 && mindist_ < 14.5) {
		prev_dist = mindist_;
		dist_tmp = mindist_;
	}
	else {
		dist_tmp = mindist_;
      		}
    	}
    }

    distance_ = dist_tmp;
 
  }   

  /*****************************/
  /* Dynamic ROI Distance Data */
  /*****************************/
  {
    std::scoped_lock lock(lane_mutex_, vel_mutex_, rep_mutex_);
    if(dist_tmp < 14.00f && dist_tmp > 3.00f) 
    {
      Lane_.cur_dist = (int)((10.0f - dist_tmp)*45.33f);
    }
    else {
      Lane_.cur_dist = 0;
    }
    Lane_.cur_vel = CurVel_;
  }
  LanePublisher_->publish(Lane_);

  if(index_ == 0) //LV
  {  
    std::scoped_lock lock(dist_mutex_, rep_mutex_);
    if(distance_ <= LVstopDist_ && distance_ > 0) {
      // Emergency Brake
    
      ResultVel_ = 0.0f;
    }
    else{
      ResultVel_ = TargetVel_;
    }
  }
  else //FVs
  {  
    std::scoped_lock lock(rep_mutex_, dist_mutex_);
    if ((distance_ <= FVstopDist_) || (TargetVel_ <= 0.1f)){
      // Emergency Brake
      ResultVel_ = 0.0f;
    }
    else {
      ResultVel_ = TargetVel_;
    }
  }
}


float ScaleTruckController::lowPassFilter(double sampling_time, float est_value, float prev_res){
  float res = 0;
  float tau = 0.01f; //0.10f
  double st = 0.0;

  if (sampling_time > 1.0) st = 1.0;
  else st = sampling_time;
  res = ((tau * prev_res) + (st * est_value)) / (tau + st);

  return res;
}

void ScaleTruckController::spin() 
{
  double diff_time=0.0;
  int cnt = 0;

  ros2_msg::msg::Xav2lrc msg;
  std::thread objectdetect_thread;

  while(!controlDone_ && rclcpp::ok()) {
    struct timeval start_time, end_time;
    static struct timeval startTime, endTime;
    static bool flag = false;
    double diffTime = 0.0;
    gettimeofday(&start_time, NULL);


    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    objectdetect_thread.join();


    msg.tar_vel = ResultVel_;  
    {
      std::scoped_lock lock(dist_mutex_);
      msg.steer_angle = AngleDegree_; 
      msg.cur_dist = distance_;       
    }

    {
      std::scoped_lock lock(rep_mutex_);
      msg.tar_dist = TargetDist_;
      msg.emergency_flag = Emergency_;
    }    

    struct timeval cur_time;
    gettimeofday(&cur_time, NULL);
    msg.stamp_sec = cur_time.tv_sec;
    msg.stamp_usec = cur_time.tv_usec;

    LrcPublisher_->publish(msg);   

    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if(!isNodeRunning_) {
      controlDone_ = true;
      rclcpp::shutdown();
    }

    if(enableConsoleOutput_)
      displayConsole();

    gettimeofday(&end_time, NULL);
    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
    cnt++;

    CycleTime_ = diff_time / (double)cnt;
//    RCLCPP_INFO(this->get_logger(), "delay Time        : %3.3f ms\n", CycleTime_);

    if (cnt > 3000){
      diff_time = 0.0;
      cnt = 0;
    }
  }
}


void ScaleTruckController::displayConsole() {
	
  fflush(stdout);
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Index             : %d", index_);
  printf("\033[2;1H");
  printf("Angle             : %2.3f degree", AngleDegree);
  printf("\033[3;1H");
  printf("k1/k2             : %3.3f %3.3f", K1_, K2_);
  printf("\033[4;1H");
  printf("e1/eL             : %3.3f %3.3f", e_values_[1], e_values_[0]);
  printf("\033[5;1H");
  printf("Tar/Cur Vel       : %3.3f / %3.3f m/s", TargetVel_, CurVel_);
  printf("\033[6;1H");
  printf("Tar/Cur Dist      : %3.3f / %3.3f m", TargetDist_, distance_);
  printf("\033[7;1H");
  printf("Cycle Time        : %3.3f ms\n", CycleTime_);
  
}


void ScaleTruckController::LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg)
{
  {
    std::scoped_lock lock(lane_mutex_);
    lane_coef_.coef = msg->coef;
    AngleDegree = msg->cur_angle;
    AngleDegree2 = msg->cur_angle2;
    center_select_ = msg->center_select;
    lc_center_follow_ = msg->lc_center_follow;
    e_values_ = msg->e_values;
    K1_ = msg->k1;
    K2_ = msg->k2;

  }
}

void ScaleTruckController::DistCallback(const ros2_msg::msg::Obj2xav::SharedPtr msg)
{
    std::scoped_lock lock(object_mutex_);
    mindist_ = msg->min_dist;
}

void ScaleTruckController::LrcSubCallback(const ros2_msg::msg::Lrc2xav::SharedPtr msg)
{
  {
    std::scoped_lock lock(vel_mutex_, rep_mutex_);
    CurVel_ = msg->cur_vel;
    TargetVel_ = msg->tar_vel;
    TargetDist_ = msg->tar_dist;
  }
}

void ScaleTruckController::CmdSubCallback(const ros2_msg::msg::Cmd2xav::SharedPtr msg)
{
  {
    std::scoped_lock lock(rep_mutex_);
    /******/
    /* LV */
    /******/
    if(index_ == 0) {   
      TargetVel_ = msg->tar_vel;
      TargetDist_ = msg->tar_dist;
      Emergency_ = msg->emergency_flag;
    }
  }

}


} /* namespace scale_truck_control */


int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<scale_truck_control::ScaleTruckController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}




