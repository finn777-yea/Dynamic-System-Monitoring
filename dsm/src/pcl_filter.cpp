#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud);

    // Define the min and max points of the box.
    Eigen::Vector4f min_point;
    min_point[0] = x_min;
    min_point[1] = y_min;
    min_point[2] = z_min;
    min_point[3] = 1.0;    // This remains 1 if we're working in Cartesian coordinates.

    Eigen::Vector4f max_point;
    max_point[0] = x_max;  // Define the values
    max_point[1] = y_max;
    max_point[2] = z_max;
    max_point[3] = 1.0;    // This remains 1 if we're working in Cartesian coordinates.

    crop.setMin(min_point);
    crop.setMax(max_point);

    // Here's the critical step: we set the filter to negative, which means we're removing the points inside the box rather than keeping them.
    crop.setNegative(true);

    // Apply the filter
    crop.filter(*cloud_filtered);

    // Now cloud_filtered is your cropped cloud, you can use it further in your processing...
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "point_cloud_listener");
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/point_cloud_topic", 1, cloudCallback);
    ros::Subscriber aabb_sub = nh.subscribe<>

    // Spin
    ros::spin();
    
    return 0;
}