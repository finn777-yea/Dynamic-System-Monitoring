#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/aabb.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dsm/aabbs.h>
#include <dsm/aabbs_array.h>
#include <dsm/bounding_box.h>
#include <dsm/bounding_box_array.h>
#include <set>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>



class compute_aabbs_node
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher aabbs_marker_pub_;
    ros::Publisher aabbs_pub_;
    ros::Publisher obbs_marker_pub_;
    ros::Publisher obbs_pub_;
    ros::Publisher bounding_box_vis_pub_;
    ros::Publisher bounding_box_array_pub_;
    ros::Subscriber joint_states_sub_;
    sensor_msgs::JointState current_joint_states_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotState robot_state_;

    public:
    compute_aabbs_node()
        : robot_model_loader_("robot_description", false),
          robot_model_(robot_model_loader_.getModel()),
          robot_state_(robot_model_)
          
    {
        robot_state_.setToDefaultValues();
        aabbs_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/aabbs_markers", 1);
        aabbs_pub_ = nh_.advertise<dsm::aabbs_array>("/aabbs", 1);
        obbs_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/obbs_markers", 1);
        bounding_box_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/bounding_box_vis", 1);
        bounding_box_array_pub_ = nh_.advertise<dsm::bounding_box_array>("/bounding_box_array", 1);
        joint_states_sub_ = nh_.subscribe("joint_states", 10, &compute_aabbs_node::jointStatesCallback, this);
    }   
    
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        current_joint_states_ = *msg;
    }
    
    void compute_aabbs()
    {
        if (!current_joint_states_.name.empty())
        {
            //Update the link_models
            
            robot_state_.setVariablePositions(current_joint_states_.name, current_joint_states_.position);
            
            //Set up a ptr vector, pointing to each link with collision geometry
            const std::vector<const moveit::core::LinkModel*>& link_models = robot_model_->getLinkModelsWithCollisionGeometry();
            std::set<std::string> names = { "base_link_inertia", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link" };
            
            visualization_msgs::Marker aabbs_marker;
            visualization_msgs::MarkerArray aabbs_marker_array;
            visualization_msgs::Marker obbs_marker;
            visualization_msgs::MarkerArray obbs_marker_array;
            visualization_msgs::Marker bbx_marker;
            dsm::aabbs_array aabbs_array;

            bbx_marker.points.clear();
            
            //Iterate through all the links, compute their aabbs respectively
            dsm::bounding_box_array boudingbox_array;
            boudingbox_array.bounding_boxes.clear();
            for(const auto link_model : link_models)
            {
                if (names.count(link_model->getName()) >0)
                {
                    
                    
                    //The global transform of the link
                    Eigen::Isometry3d transform = robot_state_.getGlobalLinkTransform(link_model);
                    //The extend of the link
                    const Eigen::Vector3d extents = link_model->getShapeExtentsAtOrigin();

                    transform.translate(link_model->getCenteredBoundingBoxOffset());
                    // std::cout << "Transform as a matrix: \n" << transform.matrix() << "\n";
                    
                    //Compute the AABB of the link_model
                    moveit::core::AABB aabb;
                    aabb.extendWithTransformedBox(transform, extents);
                
                    //Publish the max and min of the aabb
                    dsm::aabbs aabbs;
                    aabbs.max.x = aabb.max()[0];
                    aabbs.max.y = aabb.max()[1];
                    aabbs.max.z = aabb.max()[2];
                    aabbs.min.x = aabb.min()[0];
                    aabbs.min.y = aabb.min()[1];
                    aabbs.min.z = aabb.min()[2];
                    aabbs.link_name = link_model->getName();

                    
                    //Compute vertices of the obbs
                    Eigen::Quaterniond q(transform.linear());
                    
                    double ex = extents[0] / 2;
                    double ey = extents[1] / 2;
                    double ez = extents[2] / 2;

                    double xc = aabb.center()[0];
                    double yc = aabb.center()[1];
                    double zc = aabb.center()[2];
                    Eigen::Vector3d center(xc, yc, zc);

                    // Calculate the global coord. of 8 vertices and pub it
                    std::vector<Eigen::Vector4d> vertices(8);
                    vertices[0] = Eigen::Vector4d(-ex, -ey, -ez, 1);
                    // Back lower-right corner
                    vertices[1] = Eigen::Vector4d(ex, -ey, -ez, 1);
                    // Back upper-left corner
                    vertices[2] = Eigen::Vector4d(-ex, ey, -ez, 1);
                    // Back upper-right corner
                    vertices[3] = Eigen::Vector4d(ex, ey, -ez, 1);
                    // Front lower-left corner
                    vertices[4] = Eigen::Vector4d(-ex, -ey, ez, 1);
                    // Front lower-right corner
                    vertices[5] = Eigen::Vector4d(ex, -ey, ez, 1);
                    // Front upper-left corner
                    vertices[6] = Eigen::Vector4d(-ex, ey, ez, 1);
                    // Front upper-right corner
                    vertices[7] = Eigen::Vector4d(ex, ey, ez, 1);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr link_vertices_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                    for(int i = 0; i < 8; i++) {
                        // Rotate the local vertex using the quaternion.
                        Eigen::Vector4d rotated_vertex = transform * vertices[i];

                        // Translate the vertex to the box's center.
                        vertices[i] = rotated_vertex;
                        pcl::PointXYZ pcl_point;
                        geometry_msgs::Point bbx_vertices;
                        bbx_vertices.x = vertices[i](0);
                        bbx_vertices.y = vertices[i](1);
                        bbx_vertices.z = vertices[i](2);
                        pcl_point.x = vertices[i](0);
                        pcl_point.y = vertices[i](1);
                        pcl_point.z = vertices[i](2);
                        link_vertices_ptr->push_back(pcl_point);
                        bbx_marker.points.push_back(bbx_vertices);
                    }

                    // Convert bounding_box(contains 8 vertices of one link as pcl) to sensor_msg and push it back to bounding_box_array
                    dsm::bounding_box bounding_box;
                    
                    bounding_box.link_name = link_model->getName();
                    pcl::toROSMsg(*link_vertices_ptr,bounding_box.bounding_box);
                    bounding_box.bounding_box.header.frame_id = "world";
                    bounding_box.bounding_box.header.stamp = ros::Time::now();
                    
                    boudingbox_array.bounding_boxes.push_back(bounding_box);

                    // Printing the bounding box information
                    // std::cout << "OBB Vertices for link: " << bounding_box.link_name << std::endl;
                    // for(const auto& point : link_vertices_ptr->points)
                    // {
                    //     std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
                    // }

                    // Visualize the bounding box
                    bbx_marker.header.frame_id = "world";
                    bbx_marker.header.stamp = ros::Time::now();
                    bbx_marker.ns = "points";
                    bbx_marker.id = 0;
                    bbx_marker.type = visualization_msgs::Marker::POINTS;
                    bbx_marker.action = visualization_msgs::Marker::ADD;
                    bbx_marker.scale.x = 0.01;
                    bbx_marker.scale.y = 0.01;
                    bbx_marker.scale.z = 0.01;
                    bbx_marker.color.a = 1.0;
                    bbx_marker.color.r = 1.0;
                    bbx_marker.color.g = 0.0;
                    bbx_marker.color.b = 0.0;

                    
                    //Visualize the AABB of the each link
                    
                    aabbs_marker.header.frame_id = "world";
                    aabbs_marker.header.stamp = ros::Time::now();
                    aabbs_marker.ns = link_model->getName();
                    aabbs_marker.pose.position.x = aabb.center()[0];
                    aabbs_marker.pose.position.y = aabb.center()[1];
                    aabbs_marker.pose.position.z = aabb.center()[2];
                    aabbs_marker.pose.orientation.x = aabbs_marker.pose.orientation.y = aabbs_marker.pose.orientation.z = 0;
                    aabbs_marker.pose.orientation.w = 1;
                    aabbs_marker.scale.x = aabb.sizes()[0];
                    aabbs_marker.scale.y = aabb.sizes()[1];
                    aabbs_marker.scale.z = aabb.sizes()[2];
                    aabbs_marker.color.a = 0.5;
                    aabbs_marker.color.r = 0.0;
                    aabbs_marker.color.g = 0.5;
                    aabbs_marker.color.b = 0.0;
                    aabbs_marker.type = visualization_msgs::Marker::CUBE;
                    
                    aabbs_marker_array.markers.push_back(aabbs_marker);
                    aabbs_marker_pub_.publish(aabbs_marker_array);

                    //Visualize the OBB of the each link
                    
                    obbs_marker.header.frame_id = "world";
                    obbs_marker.header.stamp = ros::Time::now();
                    obbs_marker.ns = link_model->getName();
                    obbs_marker.pose.position.x = aabb.center()[0];
                    obbs_marker.pose.position.y = aabb.center()[1];
                    obbs_marker.pose.position.z = aabb.center()[2];
                   
                    obbs_marker.pose.orientation.x = q.x();
                    obbs_marker.pose.orientation.y = q.y();
                    obbs_marker.pose.orientation.z = q.z();
                    obbs_marker.pose.orientation.w = q.w();
                    obbs_marker.scale.x = extents[0];
                    obbs_marker.scale.y = extents[1];
                    obbs_marker.scale.z = extents[2];
                    obbs_marker.color.a = 0.5;
                    obbs_marker.color.r = 0.0;
                    obbs_marker.color.g = 0.0;
                    obbs_marker.color.b = 0.5;
                    obbs_marker.type = visualization_msgs::Marker::CUBE;
                    
                    obbs_marker_array.markers.push_back(obbs_marker);
                    
                }
            }
            bounding_box_array_pub_.publish(boudingbox_array);
            aabbs_pub_.publish(aabbs_array);
            obbs_marker_pub_.publish(obbs_marker_array);
            bounding_box_vis_pub_.publish(bbx_marker);
        }
    }
    
    void run()
    {
        ros::Rate loop_rate(20);
        
        while (ros::ok())
        {
            compute_aabbs();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "compute_aabb");
    compute_aabbs_node aabbs_computer;
    aabbs_computer.run();
    return 0;

}
