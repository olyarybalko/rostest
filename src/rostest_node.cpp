#include "ros/ros.h"
#include "std_msgs/String.h"
#include "point.h"
#include "systeminfo.h"
#include <sstream>



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;

  Point A, B, C;
  double d;
  A.setX(0.0);
  A.setY(0.0);
  B.setX(1.0);
  B.setY(1.0);
 
  C = A.milieu(B);
  d = A.distance(B);

  SystemInfo thisSystem;
  
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss <<  thisSystem.getDateTime() << " " << thisSystem.getSerialNumber() << " " <<  d << " " 
    << thisSystem.getUptimeSys() << " " << thisSystem.getLoadAverage(1);
    msg.data = ss.str() ;

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}