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
    DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&Planner::DistanceSubCallback, this, std::placeholders::_1));
    TargetSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("target",10,std::bind(&Planner::TargetSubCallback,this,std::placeholders::_1));
    EmergencyPublisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_brake", 10);
    VelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>("velocity",1,std::bind(&Planner::velocity_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms, std::bind(&Planner::timerCallback, this));
    TargetVelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>("target_velocity", 10);
    DistancePublisher_ = this->create_publisher<std_msgs::msg::Float32>("min_distance", 10);
    DetectedObjectsSubscriber_ = this->create_subscription<ros2_msg::msg::TrackingArray>("object_tracking/TrackingArray",1,std::bind(&Planner::DetectedObjectsSubcallback, this, std::placeholders::_1));
    LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>("lane2xav_msg",1,std::bind(&Planner::LaneSubcallback, this, std::placeholders::_1));
}

void Planner::DetectedObjectsSubcallback(const ros2_msg::msg::TrackingArray::SharedPtr msg) {
    detected_objects.clear();
    object_info info;
    for (auto &f_info : msg->trackingarr) {
        info.class_id = f_info.class_id;
        info.id = f_info.id;
        info.x = f_info.x;
        info.y = f_info.y;
        info.w = f_info.w;
        info.h = f_info.h;
        info.distance = f_info.distance;
        info.velocity = f_info.velocity;
        detected_objects.push_back(info);
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

    line_[7].at<float>(2,0) = poly_coef_.coef[4].a; // orig_lane2
    line_[7].at<float>(1,0) = poly_coef_.coef[4].b;
    line_[7].at<float>(0,0) = poly_coef_.coef[4].c;
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

bool Planner::check_collision_by_ttc() {
    object_info preceing_car;
    if(detected_objects_on_ego_lane.size() !=0 ) preceing_car = detected_objects_on_ego_lane.front();
    else return false;
    float relative_speed_mps = -preceing_car.velocity; // relative velocity
    
    if (relative_speed_mps <= 0) {
        return false;
    }
    
    float ttc = preceing_car.distance / relative_speed_mps;
    if(ttc <= 2.0){
	std::cerr << "will be collision with  "<< preceing_car.class_id << " , ttc = " << ttc << std::endl;
        return true;
    }
    else {
        return false;
    }
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
    if(current_velocity < 5.0f) des_spacing = des_spacing + 3.5f;
    spacing_err = this->current_distance - des_spacing;
    speed_err = this->target_velocity - this->current_velocity;
}

void Planner::calculate_acc_param() {
    if(detected_objects_on_ego_lane.size() != 0) this->current_distance = detected_objects_on_ego_lane.front().distance;
    else this->current_distance = 0;
    des_spacing = 1.4f *  this->current_velocity;
    spacing_err = this->current_distance - des_spacing;
    speed_err = detected_objects_on_ego_lane.front().velocity;
}

float Planner::calculate_target_velocity_cacc() {
    float t_speed = 0.0f;
    t_speed = current_velocity + myGapControlGainGap * spacing_err + myGapControlGainGapDot * speed_err;
    return t_speed;
}

float Planner::calculate_target_velocity_acc() {
    float t_speed = 0.0f;
    t_speed = current_velocity + myGapControlGainGap * spacing_err + myGapControlGainGapDot * speed_err;
    return t_speed;
}

bool Planner::isInEgoLane(const BoundingBox& box, double a, double b, double c, double ar, double br, double cr) {
    // Calculate the center point of the bounding box
    double centerX = box.x*640;
    double centerY = box.y*480;
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
                detected_objects_on_ego_lane.push_back(i);
            }
        }
        else if(cur_ego_lane == 4) {
            if(isInEgoLane(temp,line_[6].at<float>(2,0),line_[6].at<float>(1,0),line_[6].at<float>(0,0),line_[7].at<float>(2,0),line_[7].at<float>(1,0),line_[7].at<float>(0,0)))
            {
                detected_objects_on_ego_lane.push_back(i);
            }          
        }
    }
}

void Planner::timerCallback() {
    if(!lv && !is_target_trailer) {
        std::cerr << "register" << std::endl;
        register_trailer_to_follow();
    }
    if( ( this->emergency_flag  )|| (this->emergency_brake)) {
        std::cerr << "emergency!!!!!!!!!!!11 " << std::endl;
	    send_full_brake();
        return;
    }

    check_objects_ego_lane();
    sort_objects_by_distance(detected_objects_on_ego_lane);

    if(check_collision_by_ttc()) {
	std::cerr << "collision!!!!!!!!!!!!1111" << std::endl;
        std_msgs::msg::Bool msg;
        msg.data = true;
	    this->emergency_brake = true;
        EmergencyPublisher_->publish(msg);
        send_full_brake();
        return;
    }
    else {
        std_msgs::msg::Bool msg;
        msg.data = false;
        EmergencyPublisher_->publish(msg);
    }
   
    //std::cerr << detected_objects_on_ego_lane.size()  << std::endl;
    if(lv ) {//|| detected_objects_on_ego_lane.size() == 0 || detected_objects_on_ego_lane.front().id != preceding_truck_id) {
        std::cerr << "calculate_target_velocity_acc" << std::endl;
        std_msgs::msg::Float32 msg;
        if(detected_objects_on_ego_lane.size() != 0) {
            calculate_acc_param();
            msg.data = calculate_target_velocity_acc();
        }
        else {
            msg.data = this->target_velocity;
        }
        TargetVelocityPublisher_->publish(msg);
    } // ACC mode
    else {
        //    std::cerr << "calculate_target_velocity_cacc" << std::endl;
        std_msgs::msg::Float32 msg;
        calculate_cacc_param();
        msg.data = calculate_target_velocity_cacc();
        TargetVelocityPublisher_->publish(msg);
        }//CACC mode
        std_msgs::msg::Float32 msg2;
        msg2.data = this->current_distance;
        DistancePublisher_->publish(msg2);
    } 

} 



