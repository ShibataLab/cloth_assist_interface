// Record position of AR Marker by subscribing to AR Marker topic
// Author: Nishanth Koganti
// Date: 2015/8/15

#include <zmq.hpp>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/spinner.h>
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

  // AsyncSpinner is used to synchronize multiple threads under the ros framework
  // URL:http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
  ros::AsyncSpinner spinner;

  // zeromq Initialization
  zmq::context_t context(1);
  zmq::socket_t kinectSocket(context, ZMQ_PAIR);

  // create server
  kinectSocket.bind("tcp://*:5555");
  std::cout << "[ZMQ] Created kinect server" << std::endl;

  while (1)
  	{
  		// Get the file name
  		zmq::message_t request;
  		kinectSocket.recv(&request);
  		Message = std::string(static_cast<char *>(request.data()), request.size());

      if (Message == "StopServer")
  		{
  			std::cout << "[ZMQ] Received Stop Server" << std::endl;
  			break;
  		}

      else if (Message == "NewTrial")
  		{
  			std::cout << "[ZMQ] Received NewTrial" << std::endl;

  			// send ready reply
  			zmq::message_t reply(5);
  			std::memcpy((void *)reply.data(), "Ready", 5);
  			kinectSocket.send(reply);
  			std::cout << "[ZMQ] Sent Ready" << std::endl;

  			// get kinect filename
  			zmq::message_t recordName;
  			kinectSocket.recv(&recordName);
  			fileName = std::string(static_cast<char *>(recordName.data()), recordName.size());
  			std::cout << "[ZMQ] Received filename" << std::endl;

        // create and initialize file stream and write file header
        fOut.open(argv[1]);
        fOut << "Frame,Time,X,Y,Z" << std::endl;

        zmq::message_t startRequest;
  			kinectSocket.recv(&startRequest);
  			Message = std::string(static_cast<char *>(startRequest.data()), startRequest.size());
        if (Message == "StartRecording")
        {
          std::cout << "[ZMQ] Received StartRecording" << std::endl;

          // initialize frame and time variables
          begin = ros::Time::now();

          // start recording
          spinner.start();
        }
  			else
        {
          std::cout << "[ZMQ] Invalid message" << std::endl;
          continue;
        }

        // Get the file name
    		zmq::message_t stopRequest;
    		kinectSocket.recv(&stopRequest);
    		Message = std::string(static_cast<char *>(stopRequest.data()), stopRequest.size());
        if (Message == "StopRecording")
        {
          std::cout << "[ZMQ] Received StopRecording" << std::endl;
        }
  			else
        {
          std::cout << "[ZMQ] Invalid message" << std::endl;
        }

        // stop the spinner
        spinner.stop();
        fOut.close();
      }
  	}

  // clean shutdown
  ros::shutdown();
  return 0;
}
