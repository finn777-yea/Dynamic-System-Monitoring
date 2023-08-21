#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/aabb.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <dsm/aabbs.h>
#include <dsm/aabbs_array.h>
#include <set>
class compute_aabbs_node
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher aabbs_marker_pub_;
    ros::Publisher aabbs_pub_;
    ros::Publisher obbs_marker_pub_;
    ros::Publisher obbs_pub_;
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
        obbs_pub_ = nh_.advertise<dsm::aabbs_array>("/obbs", 1);
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
            dsm::aabbs_array aabbs_array;
            
            //Iterate through all the links, compute their aabbs respectively
            for(const auto link_model : link_models)
            {
                

                if (names.count(link_model->getName()) >0)
                {
                    
                    
                    //The global transform of the link
                    Eigen::Isometry3d transform = robot_state_.getGlobalLinkTransform(link_model);
                    //The extend of the link
                    const Eigen::Vector3d extents = link_model->getShapeExtentsAtOrigin();

                    transform.translate(link_model->getCenteredBoundingBoxOffset());
                    
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

                    //Add rotation infos to the msg for OBBs
                    Eigen::Quaterniond q(transform.linear());
                    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    aabbs.rotation.x = euler_angles[0];
                    aabbs.rotation.y = euler_angles[1]; 
                    aabbs.rotation.z = euler_angles[2];
                    aabbs_array.aabbs_array.push_back(aabbs);
                    

                    

                    //Log the AABB infos of the each link
                    // ROS_INFO_STREAM("Link: " << link_model->getName());
                    // ROS_INFO_STREAM("AABB Center: " << aabb.center().transpose());
                    // ROS_INFO_STREAM("AABB Sizes: " << aabb.sizes().transpose());
                    
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
                    // Eigen::Quaterniond q(transform.linear());
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
            aabbs_pub_.publish(aabbs_array);
            obbs_marker_pub_.publish(obbs_marker_array);
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
