#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include<unistd.h>  
#include <iostream>
using namespace std;

// enumeration for the different switch modes
typedef enum
{
  CmdOff_en, // switch off == 0
  CmdLocal_en, // switch from local == 1
  CmdWeb_en // switch from web == 2
}CmdSwitch_t ; 

// initializing switch and command values
CmdSwitch_t CmdSource = CmdOff_en;
geometry_msgs::Twist Cmd; 

// Callback function for switch variables
void switch_Callback(const std_msgs::Int8::ConstPtr& msg)
{
  if (msg->data ==0)
  { 
    CmdSource = CmdOff_en; // switch to off state if the received value is 0
  }
  else if (msg->data ==1)
  {
    CmdSource = CmdLocal_en; // switch to local state if the received value is 1
  }
  else if (msg->data ==2)
  {
    CmdSource = CmdWeb_en; // switch to web state if the received value is 2
  }
  else 
  {
    ROS_INFO("WRONG SWITCHER, 0 = OFF, 1 = LOCAL COMMAND , 2 = WEB COMMAND");
  }
}
// Callback function for local commands
void CmdLocal_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (CmdSource == CmdLocal_en) // process only starts when switcher is correct
  {
    ROS_INFO("COMMAND IS FROM LOCAL");
    Cmd.linear = msg->linear;
    Cmd.angular = msg->angular;
    ROS_INFO("command is [%f,%f,%f],[%f,%f,%f]", Cmd.linear.x, Cmd.linear.y, Cmd.linear.z,
                                                Cmd.angular.x, Cmd.angular.y, Cmd.angular.z);
  }
}
// Callback function for web commands
void CmdWeb_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if (CmdSource == CmdWeb_en) // process only starts when switcher is correct
  {
    ROS_INFO("COMMAND IS FROM WEB");
    Cmd.linear = msg->linear;
    Cmd.angular = msg->angular;
    ROS_INFO("command is [%f,%f,%f],[%f,%f,%f]", Cmd.linear.x, Cmd.linear.y, Cmd.linear.z,
                                      Cmd.angular.x, Cmd.angular.y, Cmd.angular.z);
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Multiplex"); // ROS node initialization

  ros::NodeHandle n; // create a node handler to interface with ROS

  // create subscribers to the switch, cmd_web, and cmd_local topics
  ros::Subscriber sub1 = n.subscribe("switch", 100, switch_Callback);
  ros::Subscriber sub2 = n.subscribe("cmd_web", 100, CmdWeb_Callback);
  ros::Subscriber sub3 = n.subscribe("cmd_local", 100, CmdLocal_Callback);

  ros::spin();

  return 0;
}