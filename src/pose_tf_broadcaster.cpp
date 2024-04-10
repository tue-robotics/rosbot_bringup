#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // y and z axis are swapped for Optitrack!
    transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ground_truth/base_link"));
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_tf_broadcaster");
    ros::NodeHandle nh;
    std::string pose_param;

    if(!nh.getParam("pose_", pose_param))
    {
        ROS_FATAL_STREAM("Parameter "
                         << "pose_"
                         << " not set");
        exit(EXIT_FAILURE);
    }

    // Subscribe to Optitrack pose topic specified by pose_param
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_param, 1, poseCallback);
    ros::spin();
    return 0;
}




