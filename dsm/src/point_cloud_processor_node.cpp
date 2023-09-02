#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/crop_hull.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <dsm/aabbs.h>
#include <dsm/aabbs_array.h>
#include <dsm/bounding_box.h>
#include <dsm/bounding_box_array.h>
#include <pcl/filters/extract_indices.h>
class PointCloudProcessorNode
{
public:
    PointCloudProcessorNode()
    {
        cloud_all_sub_ = nh_.subscribe("/cloud_concatenated", 1, &PointCloudProcessorNode::CloudAllCallback, this);
        // aabbs_sub_ = nh_.subscribe("/aabbs", 1, &PointCloudProcessorNode::aabbCallback, this);
        bounding_boxes_sub_ =nh_.subscribe("/bounding_box_array", 1, &PointCloudProcessorNode::boundingboxesCallback, this);
        
        merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_pointcloud", 1);
        merged_downsampled_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/merged_downsampled_pointcloud", 1);
        flitered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);
        hull_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/convex_hull", 1);
    }

    // void aabbCallback(const dsm::aabbs_array::ConstPtr& msg)
    // {
    //     // ROS_INFO_STREAM("Received " << msg->aabbs_array.size() << " bounding boxes.");
    //     bounding_boxes.clear();
    //     for (const auto aabbs : msg->aabbs_array)
    //     {
    //         Eigen::Vector4f min_point(aabbs.min.x, aabbs.min.y, aabbs.min.z, 1.0);
    //         Eigen::Vector4f max_point(aabbs.max.x, aabbs.max.y, aabbs.max.z, 1.0);
    //         Eigen::Vector3f rotation(aabbs.rotation.x, aabbs.rotation.y, aabbs.rotation.z);
    //         bounding_boxes.push_back(std::make_tuple(min_point, max_point, rotation));
    //         // ROS_INFO_STREAM("Bounding Box Details:");
    //         // ROS_INFO_STREAM("Link:" << aabbs.link_name);
    //         // ROS_INFO_STREAM("Min Point: [" << min_point[0] << ", " << min_point[1] << ", " << min_point[2] << "]");
    //         // ROS_INFO_STREAM("Max Point: [" << max_point[0] << ", " << max_point[1] << ", " << max_point[2] << "]");
    //         // ROS_INFO_STREAM("Rotation: [" << rotation[0] << ", " << rotation[1] << ", " << rotation[2] << "]");
    //     }
            
    
    // }
    void boundingboxesCallback(const dsm::bounding_box_array& msg)
    {
        m_boundingboxes_pcl.clear();
        // Covert each msg to pcl data and push it back to m_boundingboxes_pcl vector
        for (const auto m_box : msg.bounding_boxes)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr m_boundingbox_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(m_box.bounding_box, *m_boundingbox_pcl);
            if (m_boundingbox_pcl->points.empty())
            {
                ROS_WARN("Received an empty bounding box!");
                continue;  // Skip this iteration
            }
            m_boundingboxes_pcl.push_back(m_boundingbox_pcl);
        }
    }
    
        
    void CloudAllCallback(sensor_msgs::PointCloud2Ptr cloud_all)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_all, *cloud_all_pcl);

        // Filter out the pcl on the groud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_above_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_all_pcl);
        pass.setFilterFieldName ("z");
        pass.setNegative (true);
        pass.setFilterLimits (-1, 0.1);
        pass.filter (*cloud_all_above_pcl);

        // Downsample the merged pcl
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud_downsampled_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_all_above_pcl);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*merged_cloud_downsampled_pcl);
        sensor_msgs::PointCloud2Ptr merged_cloud_downsampled(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*merged_cloud_downsampled_pcl, *merged_cloud_downsampled);
        merged_cloud_downsampled->header.frame_id = "world";
        merged_downsampled_pub_.publish(*merged_cloud_downsampled);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        *cropped_cloud_pcl = *merged_cloud_downsampled_pcl;
        ROS_INFO("Number of points before CropHull: %zu", cropped_cloud_pcl->points.size());

        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setDimension(3);
        pcl::CropHull<pcl::PointXYZ> crophull;
        crophull.setDim(3);
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto const cropbox : m_boundingboxes_pcl)
        {
            // ROS_INFO("Number of bounding boxes: %d", m_boundingboxes_pcl.size());
            hull.setInputCloud(cropbox);
            ROS_INFO("Number of points in the cropbox: %zu", m_boundingboxes_pcl[0]->points.size());
            std::vector<pcl::Vertices> polygon;
            
            hull.reconstruct(*surface_hull ,polygon);
            // ROS_INFO("Size of the polygon vector: %zu", polygon.size());
            // ROS_INFO("Size of the surface_hull: %zu", surface_hull->points.size());
            
            // Visualize the reconstructed convex hull
            // sensor_msgs::PointCloud2 hull_msg;
            // pcl::toROSMsg(*surface_hull, hull_msg);
            // hull_msg.header.frame_id = "world";
            // hull_pub_.publish(hull_msg);
            
            crophull.setCropOutside(false);
            crophull.setInputCloud(cropped_cloud_pcl);
            crophull.setHullIndices(polygon);
            crophull.setHullCloud(surface_hull);

            crophull.filter(*cropped_cloud_pcl);
            // ROS_INFO("Number of points after CropHull: %zu", temp_cloud_pcl->points.size());
            // if (temp_cloud_pcl->points.empty()) {
            //     ROS_WARN("CropHull filtering resulted in an empty cloud!");
            //     continue;
            // }

            // *cropped_cloud_pcl = *temp_cloud_pcl;
        }
        

        
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_pcl = merged_cloud_downsampled_pcl->makeShared();
        // Filter the pcl by iterating through all the bounding boxes
        // for (const auto& box : bounding_boxes)
        // {
        //     pcl::CropBox<pcl::PointXYZ> crop;
        //     crop.setInputCloud(cropped_cloud_pcl);
        //     crop.setMin(std::get<0>(box));
        //     crop.setMax(std::get<1>(box));
        //     // Set the filter to negative, which means we're removing the points inside the box rather than keeping them.
        //     crop.setNegative(true);
        //     // Apply the filter
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //     crop.filter(*temp_cloud);
        //     cropped_cloud_pcl = temp_cloud;
        // }

        // Convert it back and publish
        sensor_msgs::PointCloud2Ptr cropped_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cropped_cloud_pcl, *cropped_cloud);
        flitered_pub_.publish(*cropped_cloud);
        ROS_INFO("Cropped pointclouds published.");

    }
    
    // void MergeAndFilter()
    // {
    //     sensor_msgs::PointCloud2Ptr merged_cloud = MergePointClouds();
    //     if (merged_cloud) 
    //     {
    //             FilterPointClouds(merged_cloud);
    //     }
    // }
    
    
    void run()
    {
        ros::AsyncSpinner spinner(4); // Use 4 threads
        spinner.start();
        // ros::spin();
        ros::waitForShutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_all_sub_;
    ros::Subscriber aabbs_sub_;
    ros::Subscriber bounding_boxes_sub_;
    ros::Publisher merged_pub_;
    ros::Publisher merged_downsampled_pub_;
    ros::Publisher hull_pub_;
    ros::Publisher cropbox_marker_array_pub_;
    ros::Publisher flitered_pub_;
    tf::TransformListener* tf_listener;

    sensor_msgs::PointCloud2Ptr cloud1_transformed_;
    sensor_msgs::PointCloud2Ptr cloud2_transformed_;
    sensor_msgs::PointCloud2Ptr cloud3_transformed_;
    sensor_msgs::PointCloud2Ptr cloud4_transformed_;

    dsm::aabbs aabb_msg;
    std::vector<std::tuple<Eigen::Vector4f, Eigen::Vector4f, Eigen::Vector3f>> bounding_boxes;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> m_boundingboxes_pcl;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processor_node");
    PointCloudProcessorNode merger;
    merger.run();
    return 0;
}