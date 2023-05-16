#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#include <math.h>

#define ANGLE_MAX 3 * M_PI_4

class LaserTransformer
{
public:
    LaserTransformer()
    {
        ros::NodeHandle nh;
        std::string laser_param;
        if (!nh.getParam("laser_", laser_param))
        {
            ROS_FATAL_STREAM("Parameter "
                             << "laser_"
                             << " not set");
            exit(EXIT_FAILURE);
        }
        laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LaserTransformer::laserCallback, this);
        laser_pub = nh.advertise<sensor_msgs::LaserScan>(laser_param, 1);
    }

private:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        sensor_msgs::LaserScan transformed_msg;

        transformed_msg.header = msg->header;
        transformed_msg.header.frame_id = "transformed_laser";
        transformed_msg.range_max = msg->range_max;
        transformed_msg.range_min = msg->range_min;
        transformed_msg.angle_max = ANGLE_MAX;
        transformed_msg.angle_min = -ANGLE_MAX;
        transformed_msg.angle_increment = msg->angle_increment;

        uint i_a_min = std::round((2 * M_PI - ANGLE_MAX) / msg->angle_increment);
        uint i_a_max = std::round(ANGLE_MAX / msg->angle_increment);
        uint n = msg->ranges.size();
        uint i_0 = n - i_a_min;
        uint n_new = std::round(2 * ANGLE_MAX / msg->angle_increment) + 1;

        ROS_INFO("index new max angle %i", i_a_max);
        ROS_INFO("index new min angle %i", i_a_min);
        ROS_INFO("original_size %i", n);
        ROS_INFO("new_size %i", n_new);

        transformed_msg.ranges.resize(n_new);
        for (int i = 0; i < i_0; i++)
        {
            // ROS_INFO("i_new: %i i_old: %i", i, i+i_a_min);
            transformed_msg.ranges[i] = msg->ranges[i + i_a_min];
        }
        for (int i = i_0; i < n_new; i++) // i=0; i<(=)i_a_max?
        {
            // ROS_INFO("i_new: %i i_old: %i", i, i-i_0);
            transformed_msg.ranges[i] = msg->ranges[i - i_0]; // i?
        }

        laser_pub.publish(transformed_msg);
    };

    ros::NodeHandle n;
    ros::Publisher laser_pub;
    ros::Subscriber laser_sub;
};

/**
 * This node transforms the laser data of Pyro to the conventions used in Mobile Robot Control
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_transformer");

    ROS_INFO("started transformer node");
    LaserTransformer laserTransformer = LaserTransformer();

    ROS_INFO("spinning");
    ros::spin();

    return 0;
}
