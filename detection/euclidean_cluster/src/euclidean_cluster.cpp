#include "euclidean_cluster.hpp"

euclidean_cluster_node::euclidean_cluster_node() : Node("radar_clustering_node") {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "front_radar", qos, std::bind(&euclidean_cluster_node::pointCloudCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_radar_points", 10);
}

euclidean_cluster_node::~euclidean_cluster_node() {}

void euclidean_cluster_node::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointXYZIV>::Ptr cloud(new pcl::PointCloud<PointXYZIV>);
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    performClustering(cloud, cluster_indices);

    publishClusters(cloud, cluster_indices);
}

void euclidean_cluster_node::performClustering(const pcl::PointCloud<PointXYZIV>::Ptr &cloud, std::vector<pcl::PointIndices> &cluster_indices) {
    pcl::search::KdTree<PointXYZIV>::Ptr tree(new pcl::search::KdTree<PointXYZIV>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointXYZIV> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

void euclidean_cluster_node::publishClusters(const pcl::PointCloud<PointXYZIV>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices) {
    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<PointXYZIV>::Ptr clustered_cloud(new pcl::PointCloud<PointXYZIV>);

    for (const auto& indices : cluster_indices) {
        for (const auto& idx : indices.indices) {
            clustered_cloud->points.push_back(cloud->points[idx]);
        }
    }

    pcl::toROSMsg(*clustered_cloud, output);
    output.header.frame_id = "laser";
    output.header.stamp = this->now();
    publisher_->publish(output);
}

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<euclidean_cluster_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
