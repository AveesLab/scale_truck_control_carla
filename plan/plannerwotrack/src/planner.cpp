#include "planner.hpp"

namespace Planner {



Planner::Planner()
       : Node("Planner", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
    this->get_parameter("truck_name", truck_name);
    if (truck_name == "truck0") lv = true;
    for (int i = 0; i < 8; i++) {
        line_.emplace_back(Mat::zeros(3, 1, CV_32F));
    }
    //DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&Planner::DistanceSubCallback, this, std::placeholders::_1));
    if(lv) TargetSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("target",10,std::bind(&Planner::TargetSubCallback,this,std::placeholders::_1));
    EmergencyPublisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_flag", 10);
    caution1Publisher_ = this->create_publisher<std_msgs::msg::Bool>("caution_mode_lane1",10);
    lcPublisher_ = this->create_publisher<std_msgs::msg::Int32>("lane_change",10);
    
    VelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>("velocity",1,std::bind(&Planner::velocity_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms, std::bind(&Planner::timerCallback, this));
    //DetectedObjectsSubscriber_ = this->create_subscription<ros2_msg::msg::FusingArray>("sensor_fusing/FusingArray",1,std::bind(&Planner::DetectedObjectsSubcallback, this, std::placeholders::_1));
    LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>("lane2xav_msg",1,std::bind(&Planner::LaneSubcallback, this, std::placeholders::_1));
    uLaneSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("ulane_change",1,std::bind(&Planner::uLaneSubcallback, this, std::placeholders::_1));
    V2xcamSubscriber_ = this->create_subscription<ros2_msg::msg::V2XCAM>("v2xcam",1,std::bind(&Planner::CAMSubcallback,this,std::placeholders::_1));
    V2xcustomSubscriber_ = this->create_subscription<ros2_msg::msg::V2XCUSTOM>("v2xcustom",1,std::bind(&Planner::CUSTOMSubcallback,this,std::placeholders::_1));
    LeftObjectSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("left_clustered_radar_points",1,std::bind(&Planner::LeftObjectSubcallback,this,std::placeholders::_1));
    RightObjectSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("right_clustered_radar_points",1,std::bind(&Planner::RightObjectSubcallback,this,std::placeholders::_1));

    TargetVelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>("target_velocity", 10);
    DistancePublisher_ = this->create_publisher<std_msgs::msg::Float32>("min_distance", 10);
    CutinFlagPublisher_ = this->create_publisher<std_msgs::msg::Bool>("cut_in_flag", 10);
    DetectedObjectsSubscriber_ = this->create_subscription<ros2_msg::msg::FusingArray>("sensor_fusing/FusingArray",1,std::bind(&Planner::DetectedObjectsSubcallback, this, std::placeholders::_1));

}

void Planner::DetectedObjectsSubcallback(const ros2_msg::msg::FusingArray::SharedPtr msg) {
    if(!received_) received_ = true;
    detected_objects.clear();
    object_info info;
    for (auto &f_info : msg->fusingarr) {
        info.class_id = f_info.class_id;
        info.id = 0;
        info.x = f_info.x;
        info.y = f_info.y;
        info.w = f_info.w;
        info.h = f_info.h;
        info.distance = f_info.distance;
        info.velocity = f_info.velocity;
        detected_objects.push_back(info);
    }
}

void Planner::LeftObjectSubcallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto data_ = msg->data;
    if(data_.size()){
        left_obstacle = true;
    }
    else {
        left_obstacle = false;
    }
}

void Planner::RightObjectSubcallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto data_ = msg->data;
    std::cerr << data_.size() << "size" << std::endl;
    if(data_.size()){
        right_obstacle = true;
    }
    else {
        right_obstacle = false;
    }
}

void Planner::LaneSubcallback(const ros2_msg::msg::Lane2xav::SharedPtr msg) {
    poly_coef_.coef = msg->coef;
    line_[0].at<float>(2,0) = poly_coef_.coef[0].a; // lane 0
    line_[0].at<float>(1,0) = poly_coef_.coef[0].b;
    line_[0].at<float>(0,0) = poly_coef_.coef[0].c;

    line_[1].at<float>(2,0) = poly_coef_.coef[1].a; // lane 1
    line_[1].at<float>(1,0) = poly_coef_.coef[1].b;
    line_[1].at<float>(0,0) = poly_coef_.coef[1].c;

    line_[2].at<float>(2,0) = poly_coef_.coef[2].a;  //center_0_1
    line_[2].at<float>(1,0) = poly_coef_.coef[2].b;
    line_[2].at<float>(0,0) = poly_coef_.coef[2].c;

    line_[3].at<float>(2,0) = poly_coef_.coef[3].a; //lane 2
    line_[3].at<float>(1,0) = poly_coef_.coef[3].b;
    line_[3].at<float>(0,0) = poly_coef_.coef[3].c;

    line_[4].at<float>(2,0) = poly_coef_.coef[4].a; // cenrter_1_2
    line_[4].at<float>(1,0) = poly_coef_.coef[4].b;
    line_[4].at<float>(0,0) = poly_coef_.coef[4].c;

    line_[5].at<float>(2,0) = poly_coef_.coef[5].a; // orig_lane0
    line_[5].at<float>(1,0) = poly_coef_.coef[5].b;
    line_[5].at<float>(0,0) = poly_coef_.coef[5].c;

    line_[6].at<float>(2,0) = poly_coef_.coef[6].a; // orig_lane1
    line_[6].at<float>(1,0) = poly_coef_.coef[6].b;
    line_[6].at<float>(0,0) = poly_coef_.coef[6].c;

    line_[7].at<float>(2,0) = poly_coef_.coef[7].a; // orig_lane2
    line_[7].at<float>(1,0) = poly_coef_.coef[7].b;
    line_[7].at<float>(0,0) = poly_coef_.coef[7].c;
}

void Planner::uLaneSubcallback(const std_msgs::msg::Int32::SharedPtr msg) {
    
    this->ulane_change = msg->data;
    
}


void Planner::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->current_velocity = msg->data;
}

void Planner::TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    this->target_velocity = msg->tar_vel;
    this->target_distance = msg->tar_dist;
    this->emergency_flag = msg->emergency_flag;
}

void Planner::DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->current_distance = msg->data;
}

void Planner::CAMSubcallback(const ros2_msg::msg::V2XCAM::SharedPtr msg) {
    this->target_velocity = static_cast<float>(msg->speed) / 100.0;
}

void Planner::CUSTOMSubcallback(const ros2_msg::msg::V2XCUSTOM::SharedPtr msg) {
    this->target_distance = msg->timegap;
    this->emergency_flag_from = msg->emergency_flag;
    if(msg->caution_mode_lane1) this->caution1mode = msg->caution_mode_lane1;
}

float Planner::check_ttc() {
    object_info preceing_car;
    if(detected_objects_on_ego_lane.size() !=0 ) preceing_car = detected_objects_on_ego_lane.front();
    else return false;
    float relative_speed_mps = -preceing_car.velocity; // relative velocity
    
    if (relative_speed_mps <= 0) {
        return false;
    }
    if(preceing_car.distance >= 990 ) return false;
    float ttc = preceing_car.distance / relative_speed_mps;
    return ttc;
    /*
    if(ttc <= 2.0){
	std::cerr << "will be collision with  "<< preceing_car.class_id << " , ttc = " << preceing_car.distance  << " " << relative_speed_mps<< std::endl;
        return true;
    }
    else {
        return false;
    }
    */
}

void Planner::send_full_brake() {
    std_msgs::msg::Float32 msg;
    msg.data = -1.0f;
    TargetVelocityPublisher_->publish(msg);
}

float Planner::calculate_target_velocity() {
    static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
    float u_dist = 0.f, u_dist_k = 0.f;
    float ref_vel = 0.f;

    dist_err = this->current_distance - this->target_distance;
    P_dist_err = this->Kp_dist_ * dist_err;
    D_dist_err = (this->Kd_dist_ * ((dist_err - prev_dist_err) / this->dt_ ));
    u_dist = P_dist_err + D_dist_err + this->target_velocity;

    // sat(u(k))  saturation start 
    if(u_dist > 90.0) u_dist_k = 90.0;
    else if(u_dist <= 0) u_dist_k = 0;
    else u_dist_k = u_dist;

    ref_vel = u_dist_k;
    prev_dist_err = dist_err;
    return ref_vel;
}

void Planner::calculate_cacc_param() {
    if(detected_objects_on_ego_lane.size() != 0) this->current_distance = detected_objects_on_ego_lane.front().distance;
    else this->current_distance = 0;
    des_spacing = this->target_distance *  this->current_velocity;
    if(current_velocity < 5.0f) des_spacing = des_spacing + 4.0f;
    spacing_err = this->current_distance - des_spacing;
    speed_err = this->target_velocity - this->current_velocity;
    //std::cerr << "lv_velocity : " << this->target_distance << std::endl;
}

void Planner::calculate_acc_param() {
    if(detected_objects_on_ego_lane.size() != 0) this->current_distance = detected_objects_on_ego_lane.front().distance;
    else this->current_distance = 0;
    des_spacing = 1.1f *  this->current_velocity;
    spacing_err = this->current_distance - des_spacing;
    speed_err = detected_objects_on_ego_lane.front().velocity;
        std::cerr << this->current_distance << " " << speed_err << std::endl;
}

float Planner::calculate_target_velocity_cacc() {
    float t_speed = 0.0f;
    t_speed = current_velocity + myGapControlGainGap * spacing_err + myGapControlGainGapDot * speed_err;
    return t_speed;
}

float Planner::calculate_target_velocity_acc() {
    float t_speed = 0.0f;
    t_speed = current_velocity + 0.23 * spacing_err + 0.07 * speed_err;
    return t_speed;
}

bool Planner::isInEgoLane(const BoundingBox& box, double a, double b, double c, double ar, double br, double cr) {
    // Calculate the center point of the bounding box
    double centerX = box.x;
    double centerY = box.y;
    //std::cerr << box.x << " " << box.w << std::endl;
    // Calculate the x coordinates of the left and right lanes at the y position of the bounding box center
    double leftLaneX = a * centerY * centerY + b * centerY + c;
    double rightLaneX = ar * centerY * centerY + br * centerY + cr;
   // std::cerr << centerX << " " << leftLaneX << " " << rightLaneX << std::endl;
    // Check if the center point is between the left and right lanes
    return centerX > leftLaneX && centerX < rightLaneX;
}

void Planner::register_trailer_to_follow() {
    int ego_lane = cur_ego_lane;
    for(auto i: detected_objects) {
        if(i.class_id == 1) {
            preceding_truck_id = i.id;
            is_target_trailer = true;
        }
    }
}

// 정렬 함수
void Planner::sort_objects_by_distance(std::vector<object_info>& objects) {
    std::sort(objects.begin(), objects.end(), [](const object_info& a, const object_info& b) {
        return a.distance < b.distance;
    });
}

void Planner::check_objects_ego_lane() {
    bool cut_in_check = false;
    int ego_lane = cur_ego_lane;
    detected_objects_on_ego_lane.clear();
    for(auto i: detected_objects) {
        BoundingBox temp;
        temp.x = i.x;
        temp.y = i.y;
        temp.h = i.h;
        temp.w = i.w;
        
        if(cur_ego_lane == 2) {
            if(isInEgoLane(temp,line_[5].at<float>(2,0),line_[5].at<float>(1,0),line_[5].at<float>(0,0),line_[6].at<float>(2,0),line_[6].at<float>(1,0),line_[6].at<float>(0,0)))
            {
                if(i.class_id == 0) cut_in_check = true;
                detected_objects_on_ego_lane.push_back(i);
            }
        }
        else if(cur_ego_lane == 4) {
            if(isInEgoLane(temp,line_[6].at<float>(2,0),line_[6].at<float>(1,0),line_[6].at<float>(0,0),line_[7].at<float>(2,0),line_[7].at<float>(1,0),line_[7].at<float>(0,0)))
            {
                if(i.class_id == 0) cut_in_check = true;
                detected_objects_on_ego_lane.push_back(i);
            }          
        }
    
        
    }
    if(cut_in_check) cut_in_flag = true;
    else cut_in_flag = false;
}


void Planner::check_ego_lane_num() {
    if(line_[1].at<float>(0,0) > 320.0) cur_ego_lane = 2;
    else cur_ego_lane = 4;


    if(lc_mode) {
        if(lc_mode != cur_ego_lane)
        {
            cur_ego_lane = lc_mode;
        }
    }
}


void Planner::lane_change_flag(int num) {
    if(num == 0) {
        lc_mode = 0;
        return;
    }
    if(num == 1) {
        std_msgs::msg::Int32 msg;
        msg.data = 2;
        lcPublisher_->publish(msg);
        lc_mode = 4;
        //cur_ego_lane = 4;
    }
    else if (num == 2) {
        std_msgs::msg::Int32 msg;
        msg.data = 4;
        lcPublisher_->publish(msg); 
        lc_mode = 2;       
        //cur_ego_lane = 2;
    }

}

bool Planner::check_side(int num) {
    if(num == 1){
        std::cerr << right_obstacle << std::endl;
        return right_obstacle;
    }
    else if(num == 2) {
        return left_obstacle;
    }
    else 
        return false;
}

void Planner::timerCallback() {
    if(!received_) {
        std::cerr << "platoon emergency mode" << std::endl;
        std_msgs::msg::Bool msg;
        msg.data = true;
        EmergencyPublisher_->publish(msg);
        send_full_brake();
        return;
    }
    if(!lv && emergency_flag_from ) {
        std::cerr << "platoon emergency mode" << std::endl;
        std_msgs::msg::Bool msg;
        msg.data = true;
        EmergencyPublisher_->publish(msg);
        send_full_brake();
        return;
    }

    if(lv && this->emergency_flag) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        EmergencyPublisher_->publish(msg);
        send_full_brake();
        return;        
    }

    check_ego_lane_num();
    check_objects_ego_lane();
    sort_objects_by_distance(detected_objects_on_ego_lane);
    float ttc_ = check_ttc();
    
    if(ttc_ <= 2.0f && ttc_ > 0) {
	    std::cerr << "collision" << ttc_ <<std::endl;
        if(!lv) {
            /*
            std_msgs::msg::Bool msg;
            msg.data = true;
            EmergencyPublisher_->publish(msg);
            send_full_brake();
            return;
            */
            if(this->ulane_change ) {
                if(check_side(this->ulane_change)){
                std_msgs::msg::Bool msg;
                msg.data = true;
                EmergencyPublisher_->publish(msg);
                send_full_brake();
            return;
                }
                else {
                    lane_change_flag(this->ulane_change); // 0: keep 1: right 2: left
                }
            }
        }

    }
    else if (ttc_ <= 3.0f){
        std_msgs::msg::Bool msg;
        msg.data = true;
        caution1Publisher_->publish(msg);
    }
    else {
        std_msgs::msg::Bool msg;
        msg.data = false;
        EmergencyPublisher_->publish(msg);
        caution1Publisher_->publish(msg);
    }

    

    if(!lv) {
        if(caution1mode) {
            if(this->ulane_change) {
                if(check_side(this->ulane_change)){
                    std_msgs::msg::Float32 msg;
                    msg.data = 0.0;
                    TargetVelocityPublisher_->publish(msg);
                    std::cerr << "full brake" << ttc_ <<std::endl;
                    return;
                }
                else {
                    lane_change_flag(this->ulane_change); // 0: keep 1: right 2: left
                    this->caution1mode = false;
                }
            }
            else {
                lc_mode = 0;
            }
        }
        else {
            if(this->ulane_change) {
                if(check_side(this->ulane_change)){
                    lv = true;
                }
                else {
                    lane_change_flag(this->ulane_change); // 0: keep 1: right 2: left
                }
            }
        }
    }

    //std::cerr << detected_objects_on_ego_lane.size() << cut_in_flag<< std::endl;
    if(lv || detected_objects_on_ego_lane.size() == 0 || cut_in_flag ) {
        std_msgs::msg::Float32 msg;
        if(detected_objects_on_ego_lane.size() != 0) {
            //std::cerr << "Check Acc1"<< std::endl;
            calculate_acc_param();
            msg.data = calculate_target_velocity_acc();
        }
        else{
                this->current_distance = 0.0f;
                //std::cerr << "Check Acc"<< std::endl;
                msg.data = 90.0;
        }
        
        TargetVelocityPublisher_->publish(msg);
    } // ACC mode
    else {
        std_msgs::msg::Float32 msg;
        calculate_cacc_param();
        msg.data = calculate_target_velocity_cacc();
        TargetVelocityPublisher_->publish(msg);
        }//CACC mode


    std_msgs::msg::Float32 msg2;
    if(this->current_distance == 999 ) msg2.data = 0.0f;
    else msg2.data = this->current_distance;

    std_msgs::msg::Bool msg3;
    if(cut_in_flag) msg3.data = true;
    else msg3.data = false;
    DistancePublisher_->publish(msg2);
    CutinFlagPublisher_->publish(msg3);
    } 

} 



/* 
    if(!lv && !is_target_trailer) {
        std::cerr << "register" << std::endl;
        register_trailer_to_follow();
    }
*/