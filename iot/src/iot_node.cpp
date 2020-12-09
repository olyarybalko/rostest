#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "send");
  ros::NodeHandle nh;
  
  // Getting params
  std::string whatToListen;
  
  nh.param<std::string>("whatToListen", whatToListen, "/chatter");

  ros::Subscriber sub = nh.subscribe(whatToListen, 1, chatterCallback);

  ros::spin();

  return 0;
}
