#include "object_tracking_node.hpp"


namespace object_tracking_ros2 {

ObjectTrackingNode::ObjectTrackingNode()
    : Node("object_tracking_node", rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{
    // if (!readParameters()) {
    //     rclcpp::shutdown();
    // }
    init();
}

ObjectTrackingNode::~ObjectTrackingNode()
{
    // delete ObjectTracker_;
}

void ObjectTrackingNode::init() {
    RCLCPP_INFO(this->get_logger(), "Tracking_Init Start");
    gettimeofday(&startTime_, NULL);

    // sub
    // std::string BboxArrayTopicName;
    // int BboxArrayQueueSize;
    std::string FusingArrayTopicName;
    int FusingArrayQueueSize;

    std::string TrackingArrayTopicName;
    int TrackingArrayQueueSize;

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();

    // this->get_parameter_or("subscribers/BboxArray/topic", BboxArrayTopicName, std::string("/yolo_object_detection/BboxArray"));
    
    this->get_parameter_or("subscribers/FusingArray/topic", FusingArrayTopicName, std::string("sensor_fusing/FusingArray"));
    this->get_parameter_or("subscribers/FusingArray/queue_size", FusingArrayQueueSize, 10);
    // BboxArraySubscriber_ = this->create_subscription<ros2_msg::msg::BboxArray>(BboxArrayTopicName, qos, std::bind(&ObjectTrackingNode::BboxArrayCallback, this, std::placeholders::_1));
    
    FusingArraySubscriber_ = this->create_subscription<ros2_msg::msg::FusingArray>("sensor_fusing/FusingArray", qos, std::bind(&ObjectTrackingNode::FusingArrayCallback, this, std::placeholders::_1));

    trackThread_ = std::thread(&ObjectTrackingNode::trackInThread, this);
    // this->get_parameter_or("publishers/TrackingArray/topic", TrackingArrayTopicName, std::string("/object_tracking/TrackingArray"));
    // this->get_parameter_or("publishers/TrackingArray/queue_size", TrackingArrayQueueSize, 1);
    TrackingPublisher_ = this->create_publisher<ros2_msg::msg::TrackingArray>("object_tracking/TrackingArray", 1);
}

// void ObjectTrackingNode::BboxArrayCallback(const ros2_msg::msg::BboxArray::SharedPtr msg) {
//     detected_objects.clear();
//     box b;
//     for (auto &box : msg->boxes) {
//         b.x = (float)box.x / 640;
//         b.y = (float)box.y / 480;
//         b.w = (float)box.w / 640;
//         b.h = (float)box.h / 480;
//         detected_objects.push_back(b);
//         // RCLCPP_INFO(this->get_logger(),"obj_box : [%f, %f, %f, %f]", b.x, b.y, b.w, b.h);
//     }
//     // std::cout << "cnt : " << cnt << std::endl;
// }


void ObjectTrackingNode::FusingArrayCallback(const ros2_msg::msg::FusingArray::SharedPtr msg) {
    fusing_objects.clear();
    int num = 0;
    ros2_msg::msg::Tracking track_;
    ros2_msg::msg::TrackingArray trackarr_;   
    object_info info;
 
    //RCLCPP_INFO(this->get_logger(), "Sub");
    for (auto &f_info : msg->fusingarr) {
        info.class_id = f_info.class_id;
        info.x = f_info.x / 640.0f;
        info.y = f_info.y / 480.0f;
        info.w = f_info.w / 640.0f;
        info.h = f_info.h / 480.0f;
        info.distance = f_info.distance;
        info.velocity = f_info.velocity;
        fusing_objects.push_back(info);
    }
    
    if(fusing_objects.size() == 0 ) {
        TrackingPublisher_->publish(trackarr_);
        return;
    }
    
    ObjectTracker_.Run(fusing_objects);
        // Visualization()
        const auto tracker = ObjectTracker_.GetTracks();
        int n  = 999;
        for (auto &trk : tracker) {
            if(trk.second.class_id == n) continue;
            else n = trk.second.class_id;
            num++;
            box b = trk.second.GetStateAsBbox();
            //RCLCPP_INFO(this->get_logger(),"obj_box : [%f, %f, %f, %f], id : %d, class_id : %d, distance : %f, velocity : %f", b.x, b.y, b.w, b.h, trk.first, trk.second.class_id, trk.second.distance, trk.second.velocity);
            track_.class_id = trk.second.class_id;
            track_.id = trk.first;
            track_.x = b.x;
            track_.y = b.y;
            track_.w = b.w;
            track_.h = b.h;
            track_.distance = trk.second.distance;
            track_.velocity = trk.second.velocity;
            trackarr_.trackingarr.push_back(track_);
        }

        TrackingPublisher_->publish(trackarr_);
        
     RCLCPP_INFO(this->get_logger(), "[%d,%d]", fusing_objects.size(),num);
}

void ObjectTrackingNode::trackInThread()
{
    RCLCPP_INFO(this->get_logger(), "trackInThread Start");
    struct timeval endTime;
    static double time = 0.0;
    while(rclcpp::ok()) {
        /*
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ros2_msg::msg::Tracking track_;
        ros2_msg::msg::TrackingArray trackarr_; 
        if(1){
        std::scoped_lock lock(fusing_objects_mutex_);
        ObjectTracker_.Run(fusing_objects);
        }
        // Visualization()
        const auto tracker = ObjectTracker_.GetTracks();
        for (auto &trk : tracker) {
            box b = trk.second.GetStateAsBbox();
            //RCLCPP_INFO(this->get_logger(),"obj_box : [%f, %f, %f, %f], id : %d, class_id : %d, distance : %f, velocity : %f", b.x, b.y, b.w, b.h, trk.first, trk.second.class_id, trk.second.distance, trk.second.velocity);
            track_.class_id = trk.second.class_id;
            track_.id = trk.first;
            track_.x = b.x;
            track_.y = b.y;
            track_.w = b.w;
            track_.h = b.h;
            track_.distance = trk.second.distance;
            track_.velocity = trk.second.velocity;
            trackarr_.trackingarr.push_back(track_);
        }
        TrackingPublisher_->publish(trackarr_);
        */
    }
}
}