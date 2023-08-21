#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"


class PCHandler
{
protected:
    ros::NodeHandle m_nh;
    sensor_msgs::PointCloud2 m_cloud;
    sensor_msgs::PointCloud2 m_cropped_cloud;
    geometry_msgs::PointStamped m_bottle_center;
    Eigen::Vector4f m_centroid;


public:
    ros::Subscriber m_pointcloud_sub, m_center_sub;
    ros::Publisher m_croppedcloud_pub, m_goalpose_pub;
    tf::TransformListener* m_tf_listener = NULL;
    tf2_ros::StaticTransformBroadcaster m_br;

PCHandler(void)
{
    m_pointcloud_sub = m_nh.subscribe("/camera/depth/color/points", 1, &PCHandler::getPC, this);
    m_center_sub = m_nh.subscribe("/point_in_3d", 1, &PCHandler::getCenter, this);

    m_goalpose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/bottle_detection/goalpose", 1);
    
    //Initialize tf_listener for pointcloud transformation
    m_tf_listener = new(tf::TransformListener);
    m_tf_listener->waitForTransform("/base_link", "/camera_depth_optical_frame", ros::Time::now(), ros::Duration(5.0));
    ROS_INFO("Transform to camera found.");
}

~PCHandler(void)
{
}
void getPC(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    m_cloud = *msg;
    ROS_INFO("successfully get PointCloud");
    m_tf_listener->waitForTransform("/base_link", m_cloud.header.frame_id, m_cloud.header.stamp, ros::Duration(3.0));
    pcl_ros::transformPointCloud("base_link", m_cloud, m_cloud, *m_tf_listener);
}

void getCenter(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    m_bottle_center = *msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(m_cloud, *pointcloud_ptr);
    ROS_INFO("size of original PointCloud: %lu", pointcloud_ptr->size());
    ////define a 3d volume around bottle
    float d = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y-d, m_bottle_center.point.z+0.3));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y-d, m_bottle_center.point.z+0.3));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y+d, m_bottle_center.point.z+0.3));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y+d, m_bottle_center.point.z+0.3));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y-d, m_bottle_center.point.z-0.1));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y-d, m_bottle_center.point.z-0.1));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x-d, m_bottle_center.point.y+d, m_bottle_center.point.z-0.1));
    boundingbox_ptr->push_back(pcl::PointXYZ(m_bottle_center.point.x+d, m_bottle_center.point.y+d, m_bottle_center.point.z-0.1));

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(boundingbox_ptr);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;
    bb_filter.setDim(3);
    bb_filter.setInputCloud(pointcloud_ptr);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(surface_hull);
    bb_filter.filter(*objects);

//test, get the maximun and minimun of a pointcloud
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*objects, minPt, maxPt);

//filter out points which belongs to the bottle cap
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(objects);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(maxPt.z-0.005, maxPt.z);
    pass.filter(*objects);

    ROS_INFO("size of cap cloud: %lu", objects->size());

//calculate centroid of bottle cap
    if(objects->size()>300){
    pcl::compute3DCentroid(*objects, m_centroid);
    ROS_INFO("center of the point cloud: %f, %f, %f", m_centroid(0), m_centroid(1), m_centroid(2));
    }
//broadcast frame of goalpose
    // static tf::TransformBroadcaster br;
    // br.sendTransform(
    //     tf::StampedTransform(
    //         tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(m_centroid[0], m_centroid[1], m_centroid[2])),
    //         ros::Time::now(), "base_link", "cap_frame"));

//publish goalpose

   tf2::Quaternion q;
   q.setRPY(-3.1415926/2, 0, 0);

   if(m_centroid(2)>=0.20 && m_centroid(2)<=0.40){
   geometry_msgs::PoseStamped goalpose;

   goalpose.pose.orientation.x = q.x();
   goalpose.pose.orientation.y = q.y();
   goalpose.pose.orientation.z = q.z();
   goalpose.pose.orientation.w = q.w();
   goalpose.pose.position.x = m_centroid[0];
   goalpose.pose.position.y = m_centroid[1];
   goalpose.pose.position.z = 0.4;
   goalpose.header.frame_id = "base_link";
   goalpose.header.stamp = ros::Time::now();

   m_goalpose_pub.publish(goalpose);
   ROS_INFO("goal pose is published!");

//Publish transform
   geometry_msgs::TransformStamped transform;
   transform.header.stamp = ros::Time::now();
   transform.header.frame_id = "base_link";
   transform.child_frame_id = "goal_position_0";
   transform.transform.translation.x = goalpose.pose.position.x;
   transform.transform.translation.y = goalpose.pose.position.y;
   transform.transform.translation.z = goalpose.pose.position.z;
   transform.transform.rotation.x = goalpose.pose.orientation.x;
   transform.transform.rotation.y = goalpose.pose.orientation.y;
   transform.transform.rotation.z = goalpose.pose.orientation.z;
   transform.transform.rotation.w = goalpose.pose.orientation.w;
   m_br.sendTransform(transform);
   }
   
//visualize the result
    // pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // centroid_ptr->push_back(pcl::PointXYZ(m_centroid(0), m_centroid(1), m_centroid(2)));
    // pcl::visualization::PCLVisualizer viewer("Centroid example");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(objects,255, 255, 255);
    // viewer.addPointCloud(objects, source_cloud_color_handler, "original_cloud");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> centroid_cloud_color_handler(centroid_ptr,230, 20, 20);
    // viewer.addPointCloud(centroid_ptr, centroid_cloud_color_handler, "cap_centroid");
    // //viewer.addCoordinateSystem(1.0, "cloud", 0);
    // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cap_centroid");
    // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    // viewer.spinOnce ();
    // }

}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_handle_node");
    ros::NodeHandle nh("~");
    PCHandler pc_handler;
    ros::spin();

    return 0;
}