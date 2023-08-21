#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>

class PointCloudProcessor
{
public:
    PointCloudProcessor()
    {
    
        camera1_sub_ = nh_.subscribe("/camera1", 1, &PointCloudMergerNode::cameraCallback, this);
        camera2_sub_ = nh_.subscribe("/camera2", 1, &PointCloudMergerNode::cameraCallback, this);
        camera3_sub_ = nh_.subscribe("/camera3", 1, &PointCloudMergerNode::cameraCallback, this);
        camera4_sub_ = nh_.subscribe("/camera4", 1, &PointCloudMergerNode::cameraCallback, this);
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera_all", 1);
    
    // Initialize the buffers
        for (int i = 0; i < 4; ++i)
        {
            point_cloud_buffers_.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
            transformations_.push_back(Eigen::Matrix4f::Identity());
        }

    // Initialize the TransformListener
        trans_listener_ = new(tf::TransformListener);
        trans_listener_->waitForTransform("world", "camera1_depth_optical_frame", ros::Time::now(), ros::Duration(10.0));
        ROS_INFO("Transform to camera1 found.");
    }
    
    // Get the tfs of the cameras
    void getCameraTransforms()
    {
        for (int i = 0; i < 4; ++i)
        {
            std::string camera_frame = "camera" + std::to_string(i+1) + "_link";
            try
            {
                tf::StampedTransform transform;
                trans_listener_->lookupTransform("world", camera_frame, ros::Time(0), transform);
                // Convert the acquired tf::StampedTransform transform to Eigen::Matrix4f and publish it
               
                    // **************

            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        ROS_INFO("Camera transforms retrieved.");
    }

    // Process the pcl data in the callbackfunction
    void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& data, int camera_index)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertToPcl(data);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformPointCloud(cloud, camera_index);
        storeInBuffer(transformed_cloud, camera_index);
        if (areAllBuffersFull())
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud = combinePointClouds();
            publishPointCloud(combined_cloud);
            clearBuffers();
        }
    }

    
    void run()
    {
        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        getCameraTransforms();
        ros::waitForShutdown();
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber camera1_sub_;
    ros::Subscriber camera2_sub_;
    ros::Subscriber camera3_sub_;
    ros::Subscriber camera4_sub_;
    ros::Publisher merged_pub_;
    ros::Publisher transformed_pub_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_buffers_;
    std::vector<Eigen::Matrix4f> transformations_; // Transformation matrices for each camera
    tf::TransformListener* trans_listener_;

    // Convert ROSmsgs to PCL::PointCloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPcl(const sensor_msgs::PointCloud2ConstPtr& data)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*data, *cloud);
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int camera_index)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transformations_[camera_index]);
        return transformed_cloud;
    }

    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_processor_node");
    PointCloudMergerNode merger;
    merger.run();
    return 0;
}
