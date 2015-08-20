// marker_recorder.cpp: Program to subscribe to ar_pose_marker and write to ros bag
// It also runs a zeromq server to connect to a synchronizer program.
// Requirements: kinect2_bridge, marker launch files running with ar_pose_marker topic available
// Author: Nishanth Koganti
// Date: 2015/8/20

// TODO:
// 1) Improve fps for recorder program (currently around 10)

// main headers
#include <fstream>
#include <cstdlib>
#include <iostream>

// zeromq header
#include <zmq.hpp>

// ros headers
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>

// ar_track_alvar header
#include <ar_track_alvar_msgs/AlvarMarkers.h>

// global variables used in the subscriber callback function
int frame;
ros::Time begin;
std::ofstream fOut;

// callback function for marker subscriber
void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  // variables for parsing subscriber message
  int id,size;
  double x,y,z;

  // get number of markers tracked
  size = msg->markers.size();

  // loop over all detected markers
  for (int i = 0; i < size; i++)
  {
    // if markerID is 4 then get markerData
    id = msg->markers[i].id;
    if (id == 4)
    {
      // get the current time
      ros::Time now = ros::Time::now();
      ros::Duration tPass = now - begin;

      // get the x,y,z positions of the marker
      x = msg->markers[i].pose.pose.position.x;
      y = msg->markers[i].pose.pose.position.y;
      z = msg->markers[i].pose.pose.position.z;

      // write data to file stream and io stream
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

  // rate variable for looping
  ros::Rate r(30);

  // there are two modes in the program
  // if input argument is only 1, then use zmq interface
  if (argc == 1)
  {
    // zeromq Initialization
    zmq::context_t context(1);
    std::string Message, fileName;
    zmq::socket_t kinectSocket(context, ZMQ_PAIR);

    // create server
    kinectSocket.bind("tcp://*:5555");
    std::cout << "[ZMQ] Created kinect server" << std::endl;

    // main loop
    while (1)
    	{
    		// Get the file name
    		zmq::message_t request;
    		kinectSocket.recv(&request);
    		Message = std::string(static_cast<char *>(request.data()), request.size());

        // stop server and clean exit if message is stop server
        if (Message == "StopServer")
    		{
    			std::cout << "[ZMQ] Received Stop Server" << std::endl;
    			break;
    		}

        // start collecting data if new trial
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

          // wait for start recording request
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

          // collect 150 data points
          while (frame < 150)
          {
            ros::spinOnce();
            r.sleep();
          }

          // send stopped recording message after 150 messages have been collected
          // get the file name
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
    // create asynchronous spinner for starting and stopping
    std::string Message;
    ros::AsyncSpinner spinner(0);

    // create and initialize file stream and write file header
    fOut.open(argv[1]);
    fOut << "Frame,Time,X,Y,Z" << std::endl;

    // initialize frame and time variables
    begin = ros::Time::now();
    spinner.start();

    // wait for input from user to stop recording
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
