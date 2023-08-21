#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <dsm/aabbs.h>
#include <dsm/aabbs_array.h>


class PointCloudProcessorNode
{
public:
    PointCloudProcessorNode()
    :   cloud1_transformed_(boost::make_shared<sensor_msgs::PointCloud2>()),
        cloud2_transformed_(boost::make_shared<sensor_msgs::PointCloud2>()),
        cloud3_transformed_(boost::make_shared<sensor_msgs::PointCloud2>()),
        cloud4_transformed_(boost::make_shared<sensor_msgs::PointCloud2>())
    {
        camera1_sub_ = nh_.subscribe("/camera1/depth/color/points", 1, &PointCloudProcessorNode::Transform1, this);
        camera2_sub_ = nh_.subscribe("/camera2/depth/color/points", 1, &PointCloudProcessorNode::Transform2, this);
        camera3_sub_ = nh_.subscribe("/camera3/depth/color/points", 1, &PointCloudProcessorNode::Transform3, this);
        camera4_sub_ = nh_.subscribe("/camera4/depth/color/points", 1, &PointCloudProcessorNode::Transform4, this);
        aabbs_sub_ = nh_.subscribe("/aabbs", 1, &PointCloudProcessorNode::aabbCallback, this);
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_pointcloud", 1);
        merged_downsampled_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_downsampled_pointcloud", 1);
        flitered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);

        // Initialize the tf_listener
        tf_listener = new(tf::TransformListener);
        tf_listener->waitForTransform("world", "/camera1_depth_optical_frame", ros::Time::now(), ros::Duration(5.0));
        // ROS_INFO("Transform to camera found.");
    }
    void Transform1(const sensor_msgs::PointCloud2ConstPtr& cloud1)
    {
    // ROS_INFO("Transform1: Executing transformation for cloud1");
    try {
        if(!cloud1)
        {
            ROS_WARN("Received null cloud pointer in Transform1.");
            return;
        }
        tf_listener->waitForTransform("world", cloud1->header.frame_id, cloud1->header.stamp, ros::Duration(3));
        pcl_ros::transformPointCloud("world", *cloud1, *cloud1_transformed_, *tf_listener);
        MergeAndFilter();
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    }

    void Transform2(const sensor_msgs::PointCloud2Ptr& cloud2)
    {
    //  ROS_INFO("Transform2: Executing transformation for cloud2");
    try {
        tf_listener->waitForTransform("world", cloud2->header.frame_id, cloud2->header.stamp, ros::Duration(3));
        pcl_ros::transformPointCloud("world", *cloud2, *cloud2_transformed_, *tf_listener);
        MergeAndFilter();
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    }

    void Transform3(const sensor_msgs::PointCloud2Ptr& cloud3)
    {
        //  ROS_INFO("Transform3: Executing transformation for cloud3");
        try {
            tf_listener->waitForTransform("world", cloud3->header.frame_id, cloud3->header.stamp, ros::Duration(3));
            pcl_ros::transformPointCloud("world", *cloud3, *cloud3_transformed_, *tf_listener);
            MergeAndFilter();
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
    }

    void Transform4(const sensor_msgs::PointCloud2Ptr& cloud4)
    {
        // ROS_INFO("Transform4: Executing transformation for cloud4");
        try {
            tf_listener->waitForTransform("world", cloud4->header.frame_id, cloud4->header.stamp, ros::Duration(3));
            pcl_ros::transformPointCloud("world", *cloud4, *cloud4_transformed_, *tf_listener);
            MergeAndFilter();
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
    }
    void aabbCallback(const dsm::aabbs_array::ConstPtr& msg)
    {
        // ROS_INFO_STREAM("Received " << msg->aabbs_array.size() << " bounding boxes.");
        bounding_boxes.clear();
        for (const auto aabbs : msg->aabbs_array)
        {
            Eigen::Vector4f min_point(aabbs.min.x, aabbs.min.y, aabbs.min.z, 1.0);
            Eigen::Vector4f max_point(aabbs.max.x, aabbs.max.y, aabbs.max.z, 1.0);
            Eigen::Vector3f rotation(aabbs.rotation.x, aabbs.rotation.y, aabbs.rotation.z);
            bounding_boxes.push_back(std::make_tuple(min_point, max_point, rotation));
            // ROS_INFO_STREAM("Bounding Box Details:");
            // ROS_INFO_STREAM("Link:" << aabbs.link_name);
            // ROS_INFO_STREAM("Min Point: [" << min_point[0] << ", " << min_point[1] << ", " << min_point[2] << "]");
            // ROS_INFO_STREAM("Max Point: [" << max_point[0] << ", " << max_point[1] << ", " << max_point[2] << "]");
            // ROS_INFO_STREAM("Rotation: [" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << "]");
        }
            
    
    }
    
    
    // Fetch the data in the each container, merge them and then publish them
    void MergeAndFilter()
    {
        

        if (cloud1_transformed_ && cloud2_transformed_ && cloud3_transformed_ && cloud4_transformed_) 
        {
            sensor_msgs::PointCloud2Ptr cloud_all(new sensor_msgs::PointCloud2);
            // ROS_INFO("All of the pointcloud available.");
            pcl::concatenatePointCloud(*cloud1_transformed_, *cloud2_transformed_, *cloud_all);
            pcl::concatenatePointCloud(*cloud_all, *cloud3_transformed_, *cloud_all);
            pcl::concatenatePointCloud(*cloud_all, *cloud4_transformed_, *cloud_all);
            cloud_all->header.stamp = ros::Time::now();
            merged_pub_.publish(*cloud_all);
            // ROS_INFO("Merged pointcloud published.");
        
            // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_all, *cloud_all_pcl);

            // Downsample the merged pcl
            pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_downsampled_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud (cloud_all_pcl);
            sor.setLeafSize (0.1f, 0.1f, 0.1f);
            sor.filter (*merged_cloud_downsampled_pcl);
            sensor_msgs::PointCloud2Ptr merged_cloud_downsampled(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*merged_cloud_downsampled_pcl, *merged_cloud_downsampled);
            merged_downsampled_pub_.publish(*merged_cloud_downsampled);

            // Iterate through all the bounding boxes
            for (const auto& box : bounding_boxes)
            {
                // Crop the cloud
                pcl::CropBox<pcl::PointXYZ> crop;
                crop.setInputCloud(cloud_all_pcl);
                crop.setMin(std::get<0>(box));
                crop.setMax(std::get<1>(box));
                // crop.setRotation(std::get<2>(box));
                // Set the filter to negative, which means we're removing the points inside the box rather than keeping them.
                crop.setNegative(true);
                // Apply the filter
                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                crop.filter(*temp_cloud);
                
                cloud_all_pcl = temp_cloud;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
            filtered_cloud_pcl = cloud_all_pcl;

            // Downsample the filetered cloud
           
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_downsampled_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            sor.setInputCloud (filtered_cloud_pcl);
            sor.setLeafSize (0.1f, 0.1f, 0.1f);
            sor.filter (*filtered_cloud_downsampled_pcl);

            // Convert it back and publish
            sensor_msgs::PointCloud2Ptr filtered_cloud_downsampled(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*filtered_cloud_downsampled_pcl, *filtered_cloud_downsampled);
            flitered_pub_.publish(*filtered_cloud_downsampled);
            ROS_INFO("Filtered pointcloud published.");
        }
        
    }
    
    
    
    void run()
    {
        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        ros::waitForShutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber camera1_sub_;
    ros::Subscriber camera2_sub_;
    ros::Subscriber camera3_sub_;
    ros::Subscriber camera4_sub_;
    ros::Subscriber aabbs_sub_;
    ros::Publisher merged_pub_;
    ros::Publisher merged_downsampled_pub_;
    ros::Publisher flitered_pub_;
    tf::TransformListener* tf_listener;

    sensor_msgs::PointCloud2Ptr cloud1_transformed_;
    sensor_msgs::PointCloud2Ptr cloud2_transformed_;
    sensor_msgs::PointCloud2Ptr cloud3_transformed_;
    sensor_msgs::PointCloud2Ptr cloud4_transformed_;

    dsm::aabbs aabb_msg;
    std::vector<std::tuple<Eigen::Vector4f, Eigen::Vector4f, Eigen::Vector3f>> bounding_boxes;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processor_node");
    PointCloudProcessorNode merger;
    merger.run();
    return 0;
}