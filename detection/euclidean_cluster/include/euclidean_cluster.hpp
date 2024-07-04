#define PCL_NO_PRECOMPILE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

struct EIGEN_ALIGN16 PointXYZIV {
  PCL_ADD_POINT4D;
  float range;
  float velocity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIV,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, range, Range)
  (float, velocity, Velocity)
)

using namespace std::chrono_literals;

class euclidean_cluster_node : public rclcpp::Node
{
public:
    explicit euclidean_cluster_node();
    ~euclidean_cluster_node();

private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void performClustering(const pcl::PointCloud<PointXYZIV>::Ptr &cloud, std::vector<pcl::PointIndices> &cluster_indices);
    void publishClusters(const pcl::PointCloud<PointXYZIV>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices);
};

