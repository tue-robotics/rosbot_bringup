#include "ros/ros.h"

#include "sensor_msgs/Range.h" 
#include "std_msgs/Bool.h" 

#define BUMPERSIZE 0.09 //

class BumperEmulator
{
   public:
   BumperEmulator()
   {
      ros::NodeHandle nh;
      std::string bumper_f_param, bumper_b_param;
      if (!nh.getParam("bumper_f_", bumper_f_param))
      {
         ROS_FATAL_STREAM("Parameter " << "bumper_f_" << " not set");
         exit(EXIT_FAILURE);
      }
      if (!nh.getParam("bumper_b_", bumper_b_param))
      {
         ROS_FATAL_STREAM("Parameter " << "bumper_b_" << " not set");
         exit(EXIT_FAILURE);
      }

        
      range_fr_sub = nh.subscribe<sensor_msgs::Range>("/range/fr", 1, &BumperEmulator::frRangeCallback, this);
      range_fl_sub = nh.subscribe<sensor_msgs::Range>("/range/fl", 1, &BumperEmulator::flRangeCallback, this);
      range_rr_sub = nh.subscribe<sensor_msgs::Range>("/range/rr", 1, &BumperEmulator::rrRangeCallback, this);
      range_rl_sub = nh.subscribe<sensor_msgs::Range>("/range/rl", 1, &BumperEmulator::rlRangeCallback, this);
	 
      bumper_f_pub = nh.advertise<std_msgs::Bool>(bumper_f_param, 1);
      bumper_r_pub = nh.advertise<std_msgs::Bool>(bumper_b_param, 1);
   }
   
   void PublishBumperData()
   {
   	std_msgs::Bool msg_f;
   	msg_f.data = f_contact;
   	bumper_f_pub.publish(msg_f);
   	
   	std_msgs::Bool msg_r;
   	msg_r.data = r_contact;
   	bumper_r_pub.publish(msg_r);
   }
	
   private:
   
   void frRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	fr_contact = (msg-> range <= BUMPERSIZE);
   	f_contact = fr_contact || fl_contact;
   }
   
   void flRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	fl_contact = (msg-> range <= BUMPERSIZE);
   	f_contact = fr_contact || fl_contact;
   }
   
   void rrRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	rr_contact = (msg-> range <= BUMPERSIZE);
   	r_contact = rr_contact || rl_contact;
   }
   
   void rlRangeCallback(const sensor_msgs::Range::ConstPtr& msg)
   {
   	rl_contact = (msg-> range <= BUMPERSIZE);
   	r_contact = rr_contact || rl_contact;
   }
   
   bool fr_contact;
   bool fl_contact;
   bool rr_contact;
   bool rl_contact;
   
   bool f_contact;
   bool r_contact;
   
   ros::Publisher bumper_f_pub;
   ros::Publisher bumper_r_pub;
   
   ros::Subscriber range_fr_sub;
   ros::Subscriber range_fl_sub;
   ros::Subscriber range_rr_sub;
   ros::Subscriber range_rl_sub;   
};



/**
 * This node emulates a (boolean) bumper, using the range sensors present on Pyro
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv,"bumper_emulator");
  BumperEmulator bumperEmulator = BumperEmulator();
	
  ros::Rate loop_rate(10); // 10 Hz
  ROS_INFO("started bumper emulator node");

  while (ros::ok())
  {
     ros::spinOnce();
     bumperEmulator.PublishBumperData();
     loop_rate.sleep();
  }
  return 0;
}
