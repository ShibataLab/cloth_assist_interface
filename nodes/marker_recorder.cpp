// Record position of AR Marker by subscribing to AR Marker topic
// Author: Nishanth Koganti
// Date: 2015/8/15

#include <zmq.hpp>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

int frame;
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
  ros::Subscriber sub = n.subscribe("ar_pose_marker", 0, callback);

  if (argc == 1)
  {
    // zeromq Initialization
    zmq::context_t context(1);
    std::string Message, fileName;
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
          fOut.open(fileName);
          fOut << "Frame,Time,X,Y,Z" << std::endl;
          std::cout << "[FSTREAM] Opened file and written header" << std::endl;

          zmq::message_t startRequest;
    			kinectSocket.recv(&startRequest);
    			Message = std::string(static_cast<char *>(startRequest.data()), startRequest.size());
          if (Message == "StartRecording")
          {
            std::cout << "[ZMQ] Received StartRecording" << std::endl;

            // initialize frame and time variables
            frame = 0;
            begin = ros::Time::now();
          }
    			else
          {
            std::cout << "[ZMQ] Invalid message" << std::endl;
            continue;
          }

          ros::Rate r(30);

          while (frame < 150)
          {
            ros::spinOnce();
            r.sleep();
          }

          // Get the file name
      		zmq::message_t stopRequest(16);
          std::memcpy((void *)stopRequest.data(), "StoppedRecording", 16);
    			kinectSocket.send(stopRequest);
    			std::cout << "[ZMQ] Sent StoppedRecording" << std::endl;

          // stop the spinner
          fOut.close();

          std::cout << "[FSTREAM] Closed file and stopped recording" << std::endl;
        }
    	}
  }
  else
  {
    std::string Message;

    ros::AsyncSpinner spinner(0);

    // create and initialize file stream and write file header
    fOut.open(argv[1]);
    fOut << "Frame,Time,X,Y,Z" << std::endl;

    // initialize frame and time variables
    begin = ros::Time::now();
    spinner.start();

    std::cout << "Stop recording? Y" << std::endl;
    std::cin >> Message;

    // stop recording and close fstream
    spinner.stop();
    fOut.close();
  }

  // clean shutdown
  ros::shutdown();
  return 0;
}
