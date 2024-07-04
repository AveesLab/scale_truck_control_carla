#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_msg/msg/gnss.hpp"
#include "ros2_msg/msg/v2_xcam.hpp"
#include "ros2_msg/msg/imu.hpp"
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Geo2Location class
class Geo2Location {
public:
    Geo2Location() {
        // Predefined geolocation points based on provided data
        Eigen::Matrix4d l;
        l << 0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1,
             1, 1, 1, 1;
             
        Eigen::Matrix4d g;
        g << -0.002071, -0.002071, -0.002080, -0.002071,
             0.001897, 0.001906, 0.001897, 0.001897,
             0.0, 0.0, 0.0, 1.0,
             1, 1, 1, 1;

        _tform = (g * l.inverse()).inverse();
    }

    Eigen::Vector3d transform(double latitude, double longitude, double altitude) {
        Eigen::Vector4d geoloc(latitude, longitude, altitude, 1.0);
        Eigen::Vector4d loc = _tform * geoloc;
        return Eigen::Vector3d(loc[0], loc[1], loc[2]);
    }

private:
    Eigen::Matrix4d _tform;
};

// GNSSConverter node class
class GNSSConverter : public rclcpp::Node {
public:
    GNSSConverter() : Node("gnss_converter"), geo2loc_() {
        gnss_sub_1_.subscribe(this, "gnss");
        gnss_sub_2_.subscribe(this, "v2xcam");
        imu_sub_3_.subscribe(this,"imu");

        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), gnss_sub_1_, gnss_sub_2_,imu_sub_3_);
        sync_->registerCallback(std::bind(&GNSSConverter::gnss_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        xyz_pub_ = this->create_publisher<std_msgs::msg::Int32>("ulane_change", 10);
    }

private:
    const double PI = 3.14159265358979323846;
    void gnss_callback(const ros2_msg::msg::GNSS::ConstSharedPtr msg1, const ros2_msg::msg::V2XCAM::ConstSharedPtr msg2, const ros2_msg::msg::IMU::ConstSharedPtr msg3) {
        auto location1 = geo2loc_.transform(msg1->latitude, msg1->longitude, msg1->altitude);
        double scale = 1e-7;
        double lat_double = msg2->latitude * scale;
        double lon_double = msg2->longitude * scale;
        auto location2 = geo2loc_.transform(lat_double, lon_double, msg1->altitude);

        //geometry_msgs::msg::Point point_msg;

        //std::cerr << "loc1" <<location1.x() << " " << location1.y() << " " << location1.z() << std::endl;
        //std::cerr << location2.x() << " " << location2.y() << " " << location2.z() << std::endl;

        double deltaX = location2.x() - location1.x();
        double deltaY = location2.y() - location1.y();
        double theta = atan2(deltaY, deltaX) * 180.0 / PI;
        //std::cerr << theta << std::endl;
        
        if(std::abs(theta) > 1.0) {
            if(theta > 0.0) {
                std::cerr << "preceding vehicle is in right lane " << std::endl;
                std_msgs::msg::Int32 msg;
                msg.data = 1;
                xyz_pub_->publish(msg);
            }
            else {
                std::cerr << "preceding vehicle is in left lane " << std::endl;
                std_msgs::msg::Int32 msg;
                msg.data = 2;
                xyz_pub_->publish(msg);
            }
        }
        else {
            std::cerr << "preceding vehicle is in same lane " << std::endl;
            std_msgs::msg::Int32 msg;
            msg.data = 0;
            xyz_pub_->publish(msg);
        }

        //xyz_pub_->publish(point_msg);
    }

    typedef message_filters::sync_policies::ApproximateTime<ros2_msg::msg::GNSS, ros2_msg::msg::V2XCAM, ros2_msg::msg::IMU> MySyncPolicy;
    message_filters::Subscriber<ros2_msg::msg::GNSS> gnss_sub_1_;
    message_filters::Subscriber<ros2_msg::msg::V2XCAM> gnss_sub_2_;
    message_filters::Subscriber<ros2_msg::msg::IMU> imu_sub_3_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr xyz_pub_;
    Geo2Location geo2loc_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSConverter>());
    rclcpp::shutdown();
    return 0;
}
