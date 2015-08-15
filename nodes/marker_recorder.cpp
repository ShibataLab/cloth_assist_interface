// Record position of AR Marker by subscribing to AR Marker topic
// Author: Nishanth Koganti
// Date: 2015/8/15

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

int frame = 0;
ros::Time begin;
std::ofstream fOut;

void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  int id,size;
  double x,y,z;

  size = msg->markers.size();
  for (int i = 0; i < size; i++)
  {
    id = msg->markers[i].id;
    if (id == 4)
    {
      ros::Time now = ros::Time::now();
      ros::Duration tPass = now - begin;

      x = msg->markers[i].pose.pose.position.x;
      y = msg->markers[i].pose.pose.position.y;
      z = msg->markers[i].pose.pose.position.z;

      fOut << frame << "," << tPass.toSec() << "," << x << "," << y << "," << z << std::endl;
      std::cout << frame << "," << tPass.toSec() << "," << x << "," << y << "," << z << std::endl;
      frame++;
    }
  }
}

int main(int argc, char **argv)
{
  // initialize ros node
  ros::init(argc, argv, "ARMarkerRecorder");

  // ros node handle
  ros::NodeHandle n;

  // subscribe to ros topic
  ros::Subscriber sub = n.subscribe("ar_pose_marker", 50, callback);

  // create and initialize file stream
  fOut.open(argv[1]);

  // initialize frame and time variables
  ros::Rate r(30);
  begin = ros::Time::now();

  // starting ros subscriber
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  fOut.close();
  return 0;
}
