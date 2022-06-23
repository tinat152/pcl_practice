#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

ros::Publisher pub;

float z_min_filter_limit;
float z_max_filter_limit;
float distanceThreshold;
std::string filterFieldName;
float pointColorThreshold;
float regionColorThreshold;
float minClusterSize;
float leafSizeX;
float leafSizeY;
float leafSizeZ;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (raw_cloud);
    sor.setLeafSize (leafSizeX, leafSizeY, leafSizeZ);
    sor.filter (*transformed_cloud);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (transformed_cloud);
    pass.setFilterFieldName (filterFieldName);
    pass.setFilterLimits (z_min_filter_limit, z_max_filter_limit);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (transformed_cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (distanceThreshold);
    reg.setPointColorThreshold (pointColorThreshold);
    reg.setRegionColorThreshold (regionColorThreshold);
    reg.setMinClusterSize (minClusterSize);

    /*
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    transformed_cloud = reg.getColoredCloud();
    */

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);

    output.header.frame_id = cloud_msg->header.frame_id;
    output.header.stamp = cloud_msg->header.stamp;

    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_bootcamp");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, cloud_cb);

    pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);

    ros::Rate loop_rate(2);

    while(ros::ok()) {
        n.param<float>("z_min_filter_limit", z_min_filter_limit, 0.0);
        n.param<float>("z_max_filter_limit", z_max_filter_limit, 1.0);
        n.param<float>("distanceThreshold", distanceThreshold, 10);
        n.param<std::string>("filterFieldName", filterFieldName, "z");
        n.param<float>("pointColorThreshold", pointColorThreshold, 6.0);
        n.param<float>("regionColorThreshold", regionColorThreshold, 5.0);
        n.param<float>("minClusterSize", minClusterSize, 600.0);
        n.param<float>("leafSizeX", leafSizeX, 0.05f);
        n.param<float>("leafSizeY", leafSizeY, 0.05f);
        n.param<float>("leafSizeZ", leafSizeZ, 0.05f);

        ros::spinOnce();
        loop_rate.sleep();
    }
}