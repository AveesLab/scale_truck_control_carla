#include "object_tracking_node.hpp"
#include "tracker.h"


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

     // Initialize SORT

    // sub
    std::string BboxArrayTopicName;
    int BboxArrayQueueSize;
    
    // pub
    // std::string trackingInfoTopicName;
    // int trackingInfoQueueSize;
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();

    this->get_parameter_or("subscribers/BboxArray/topic", BboxArrayTopicName, std::string("/yolo_object_detection/BboxArray"));
    // RADAR
    // this->get_parameter_or("subscribers/RadarPointCloud/topic", RadarPointCloudTopicName);
    /************************/
    /* Ros Topic Subscriber */
    /************************/
    BboxArraySubscriber_ = this->create_subscription<ros2_msg::msg::BboxArray>(BboxArrayTopicName, qos, std::bind(&ObjectTrackingNode::BboxArrayCallback, this, std::placeholders::_1));
    

    /***********************/
    /* Ros Topic Publisher */
    /***********************/
    // frontTrackingSubscriber_ = this->create_publisher<ros2_msg::msg::>();

    /***************/
    /* Start Setup */
    /***************/
    // cv::Mat img_for_init(width_, height_, CV_8UC3, cv::Scalar(0, 0, 0));

    
    trackThread_ = std::thread(&ObjectTrackingNode::trackInThread, this);
    
}

void ObjectTrackingNode::BboxArrayCallback(const ros2_msg::msg::BboxArray::SharedPtr msg) {
    detected_objects.clear();
    box b;
    int cnt = 0;
    for (auto &box : msg->boxes) {
        b.x = (float)box.x / 640;
        b.y = (float)box.y / 480;
        b.w = (float)box.w / 640;
        b.h = (float)box.h / 480;
        detected_objects.push_back(b);
        RCLCPP_INFO(this->get_logger(),"obj_box : [%f, %f, %f, %f]", b.x, b.y, b.w, b.h);
    }
    // std::cout << "cnt : " << cnt << std::endl;
}

// void ObjectTrackingNode::RadarCallback(const )

// void ObjectTrackingNode::Run() {
//     tracker.Run()
// }

// Option
// void ObjectTrackingNode::Visualization(cv::Mat mat_img, Tracker tracker) {
//     const auto tracks = tracker.GetTracks();
    
//     for (auto &trk : tracks) {
//         //// edit 
//         box b = trk.second.GetStateAsBbox();
//         int width = std::max(1.0f, show_img->rows * .002f);
//         int left = (b.x - b.w / 2.)*show_img->cols;
//         int right = (b.x + b.w / 2.)*show_img->cols;
//         int top = (b.y - b.h / 2.)*show_img->rows;
//         int bot = (b.y + b.h / 2.)*show_img->rows;
        
//         float const font_size = show_img->rows / 100.F;
//     //     // cv::Size const text_size = cv::getTextSize(labelstr, cv::FONT_HERSHEY_COMPLEX_SMALL, font_size, 1, 0);
//         cv::Point pt_text, pt_text_bg1, pt_text_bg2, pt1, pt2;
//         pt1.x = left;
//         pt1.y = top;
//         pt2.x = right;
//         pt2.y = bot;

//         pt_text.x = right;
//         pt_text.y = top - 4;// 12;
//         pt_text_bg1.x = left;
//         pt_text_bg1.y = bot;
//         pt_text_bg2.x = right;
//         // if ((right - left) < text_size.width) pt_text_bg2.x = left + text_size.width;
//         // cv::Scalar color = CV_RGB(0, 255, 0);
//         pt_text_bg2.y = bot+ (3 + 18 * font_size);
//     //     // cv::rectangle(*show_img, pt_text_bg1, pt_text_bg2, color, width, 8, 0);
//     //     // cv::rectangle(*show_img, pt_text_bg1, pt_text_bg2, color, CV_FILLED, 8, 0);
//         cv::Scalar black_color = CV_RGB(0, 0, 0);
//         //cv::rectangle(*show_img, pt1, pt2, black_color, width, 8, 0);
//         cv::putText(*show_img, std::to_string(trk.first), pt_text, cv::FONT_HERSHEY_COMPLEX_SMALL, font_size, black_color, 2 * font_size, CV_AA);
//         }
//     }

// void ObjectTrackingNode::publishInThread()
// {
//     // ros2_msg::msg::TrackingInfo msg;
// }



void ObjectTrackingNode::trackInThread()
{
    RCLCPP_INFO(this->get_logger(), "trackInThread Start");
    struct timeval endTime;
    static double time = 0.0;
    Tracker ObjectTracker_ = Tracker();
    while(rclcpp::ok()) {
        // ObjectTracker_.Run(detected_objects);
        // Visualization()
        ObjectTracker_.Run(detected_objects);
        const auto tracker = ObjectTracker_.GetTracks();
        for (auto &trk : tracker) {
            box b = trk.second.GetStateAsBbox();
            RCLCPP_INFO(this->get_logger(),"obj_box : [%f, %f, %f, %f], id : %d", b.x, b.y, b.w, b.h, trk.first);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
}