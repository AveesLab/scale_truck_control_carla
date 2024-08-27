#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarVehicleDetection : public rclcpp::Node {
public:
    LidarVehicleDetection()
        : Node("lidar_vehicle_detection") {
        // Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/truck1/lidar0", rclcpp::SensorDataQoS(),
            std::bind(&LidarVehicleDetection::processLidarData, this, std::placeholders::_1));

        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/truck1/detected_vehicles", 10);
    }

private:
    void processLidarData(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Voxel grid filter for downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*cloud_filtered);

        // Plane segmentation to remove ground plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_no_ground);

        // Clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(cloud_no_ground);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_no_ground);
        ec.extract(cluster_indices);

        pcl::PointCloud<pcl::PointXYZ>::Ptr vehicle_clusters(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());
            for (const auto& idx : indices.indices) {
                cloud_cluster->push_back((*cloud_no_ground)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            Eigen::Vector4f min_point, max_point;
            pcl::getMinMax3D(*cloud_cluster, min_point, max_point);
            float cluster_length = max_point.x() - min_point.x();
            float cluster_width = max_point.y() - min_point.y();
            float cluster_height = max_point.z() - min_point.z();

            if (cluster_length >= 0.0 && cluster_length < 9.5 &&
                cluster_width > 0.5 && cluster_width < 9.0 &&
                cluster_height > 0.1 && cluster_height < 5.5) {
                *vehicle_clusters += *cloud_cluster;
            }
        }

        // Publish detected vehicles
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*vehicle_clusters, output_msg);
        output_msg.header.stamp = rclcpp::Clock().now();
        output_msg.header.frame_id = "laser";
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarVehicleDetection>());
    rclcpp::shutdown();
    return 0;
}
