#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud_pcl);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud_pcl, "voxelized cloud", 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "voxelized cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "voxelized cloud");
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxelization_node");
    ros::NodeHandle nh;

    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1, cloudCallback);

    ros::spin();
    return 0;
}