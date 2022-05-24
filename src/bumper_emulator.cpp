#include "ros/ros.h"

#include "sensor_msgs/Range.h" 
#include "std_msgs/Bool.h" 

#define BUMPERSIZE 1 //todo

class BumperEmulator
{
    public:
    BumperEmulator()
    {
        ros::NodeHandle nh;
        
        range_fr_sub = nh.subscribe<sensor_msgs::Range>("/range/fr", 1, &BumperEmulator::frRangeCallback, this);
        range_fl_sub = nh.subscribe<sensor_msgs::Range>("/range/fl", 1, &BumperEmulator::flRangeCallback, this);
        range_br_sub = nh.subscribe<sensor_msgs::Range>("/range/br", 1, &BumperEmulator::brRangeCallback, this);
        range_bl_sub = nh.subscribe<sensor_msgs::Range>("/range/bl", 1, &BumperEmulator::blRangeCallback, this);
	 
	bumper_f_pub = nh.advertise<std_msgs::Bool>("/bumper_todo_f", 1);
	bumper_b_pub = nh.advertise<std_msgs::Bool>("/bumper_todo_b", 1);
   }
   
   private:
   
   void PublishBumperData()
   {
   	std_msgs::Bool msg_f;
   	msg_f.data = f_contact;
   	bumper_f_pub.publish(msg_f);
   }
   
   void frRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	fr_contact = (msg-> range <= BUMPERSIZE);
   	f_contact = fr_contact || fl_contact;
   	
   	PublishBumperData();
   }
   
   void flRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	fl_contact = (msg-> range <= BUMPERSIZE);
   	f_contact = fr_contact || fl_contact;
   	
   	PublishBumperData();
   }
   
   void brRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	br_contact = (msg-> range <= BUMPERSIZE);
   	b_contact = br_contact || bl_contact;
   }
   
   void blRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	bl_contact = (msg-> range <= BUMPERSIZE);
   	b_contact = br_contact || bl_contact;
   }
   
   bool fr_contact;
   bool fl_contact;
   bool br_contact;
   bool bl_contact;
   
   bool f_contact;
   bool b_contact;
   
   ros::Publisher bumper_f_pub;
   ros::Publisher bumper_b_pub;
   
   ros::Subscriber range_fr_sub;
   ros::Subscriber range_fl_sub;
   ros::Subscriber range_br_sub;
   ros::Subscriber range_bl_sub;   
};



/**
 * This node emulates a (boolean) bumper, using the range sensors present on Pyro
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv,"bumper_emulator");

  ROS_INFO("started bumper emulator node");
  BumperEmulator bumperEmulator = BumperEmulator();

  ROS_INFO("spinning");
  ros::spin();

  return 0;
}
