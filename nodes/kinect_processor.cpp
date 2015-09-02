// kinect_processor.cpp: Program to read rosbag files and process kinect2 data
// Requirements: rosbag file as input
// Author: Nishanth Koganti
// Date: 2015/9/2

// CPP headers
#include <cmath>
#include <string>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>

// ROS headers
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// ROS messaging filters
#include <message_filters/subscriber.h>

// kinect2 bridge header
#include <kinect2_bridge/kinect2_definitions.h>

// help function
void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  mode: 'qhd', 'hd', 'sd'" << std::endl;
}

int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "kinect_processor", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // ns = kinect2
  // selecting default topic names when the options are not provided
  std::string topicType = "sd";
  std::string ns = K2_DEFAULT_NS;
  std::string fileName = "kinect";
  std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

  // parsing command line arguments
  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    // printing help information
    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }

    // selecting quater hd topics
    else if(param == "qhd")
    {
      topicType = param;
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }

    // selecting hd topics
    else if(param == "hd")
    {
      topicType = param;
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }

    // selecting depth topics
    else if(param == "sd")
    {
      topicType = param;
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
  }

  // initializing color and depth topic names
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::string topicCameraInfo = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;
  std::cout << "topic camera info: " << topicCameraInfo << std::endl;

  // start running processor instance
  std::cout << "starting recorder..." << std::endl;
  recorder.run();

  // clean shutdown
  ros::shutdown();
  return 0;
}





rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Read);

std::vector<std::string> topics;
topics.push_back(std::string("chatter"));
topics.push_back(std::string("numbers"));

rosbag::View view(bag, rosbag::TopicQuery(topics));

foreach(rosbag::MessageInstance const m, view)
{
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
    if (s != NULL)
        ASSERT_EQ(s->data, std::string("foo"));

    std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
    if (i != NULL)
        ASSERT_EQ(i->data, 42);
}

bag.close();
