#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_planning_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/pos_joint_traj_controller/command", 10);

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.joint_names.push_back("shoulder_lift_joint");
    trajectory.joint_names.push_back("elbow_joint");
    trajectory.joint_names.push_back("shoulder_pan_joint");
    trajectory.joint_names.push_back("wrist_1_joint");
    trajectory.joint_names.push_back("wrist_2_joint");
    trajectory.joint_names.push_back("wrist_3_joint");
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(-2.6); // position for shoulder_lift_joint
    point.positions.push_back(1.3); // position for elbow_joint
    point.positions.push_back(-5.2);
    point.positions.push_back(-2.0);
    point.positions.push_back(0.1);
    point.positions.push_back(0.1);



    point.time_from_start = ros::Duration(3.0); // 1 second to reach the goal
    trajectory.points.push_back(point);

    ros::Rate rate(10); // 10Hz

    while(ros::ok())
    {
        pub.publish(trajectory);
        rate.sleep();
    }

    return 0;
}