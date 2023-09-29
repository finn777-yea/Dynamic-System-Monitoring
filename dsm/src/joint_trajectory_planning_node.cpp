#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_planning_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 3);

    trajectory_msgs::JointTrajectory trajectory1;
    trajectory1.header.stamp = ros::Time::now();
    trajectory1.joint_names.push_back("shoulder_lift_joint");
    trajectory1.joint_names.push_back("elbow_joint");
    trajectory1.joint_names.push_back("shoulder_pan_joint");
    trajectory1.joint_names.push_back("wrist_1_joint");
    trajectory1.joint_names.push_back("wrist_2_joint");
    trajectory1.joint_names.push_back("wrist_3_joint");
    
    trajectory_msgs::JointTrajectoryPoint point1;
    point1.positions.push_back(-2.6); // position for shoulder_lift_joint
    point1.positions.push_back(1.3); // position for elbow_joint
    point1.positions.push_back(-5.2);
    point1.positions.push_back(-2.0);
    point1.positions.push_back(1.2);
    point1.positions.push_back(0.8);

    point1.time_from_start = ros::Duration(10);
    trajectory1.points.push_back(point1);

    trajectory_msgs::JointTrajectory trajectory2;
    trajectory2.header.stamp = ros::Time::now();
    trajectory2.joint_names.push_back("shoulder_lift_joint");
    trajectory2.joint_names.push_back("elbow_joint");
    trajectory2.joint_names.push_back("shoulder_pan_joint");
    trajectory2.joint_names.push_back("wrist_1_joint");
    trajectory2.joint_names.push_back("wrist_2_joint");
    trajectory2.joint_names.push_back("wrist_3_joint");
    
    trajectory_msgs::JointTrajectoryPoint point2;
    point2.positions.push_back(0); // position for shoulder_lift_joint
    point2.positions.push_back(0); // position for elbow_joint
    point2.positions.push_back(0);
    point2.positions.push_back(0);
    point2.positions.push_back(0);
    point2.positions.push_back(0);

    point2.time_from_start = ros::Duration(10);
    trajectory2.points.push_back(point2);

    ros::Rate rate(1); // 10Hz

    while(ros::ok())
    {
        pub.publish(trajectory1);
        ros::Duration(10).sleep();
        pub.publish(trajectory2);
        ros::Duration(10).sleep();
    }

    return 0;
}