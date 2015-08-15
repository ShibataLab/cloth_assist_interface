// Record position of AR Marker by subscribing to AR Marker topic
// Author: Nishanth Koganti
// Date: 2015/8/15

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>

int frame;
ros::Time begin;

void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  double x,y,z;
  int id, size;
  size = msg->markers.size();

  for (int i = 0; i < size; i++)
  {
    id = msg->markers[i].id;

    if (id == 4)
    {
      frame++;
      ros::Time now = ros::Time::now();
      ros::Duration tPass = now - begin;

      x = msg->markers[i].pose.pose.position.x;
      y = msg->markers[i].pose.pose.position.y;
      z = msg->markers[i].pose.pose.position.z;
      
      std::cout << frame << " " << tPass.toSec() << " " << x << " " << y << " " << z << std::endl;
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
  ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, callback);

  frame = 0;
  begin = ros::Time::now();

  // starting ros subscriber
  ros::spin();

  return 0;
}
