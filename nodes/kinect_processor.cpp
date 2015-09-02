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
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Boost headers
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

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
  std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

  std::string flag = "y";
  std::string fileName = "default";

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

    // other option can only be the fileName
    else
    {
      fileName = param;
    }
  }

  // initializing color and depth topic names
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::string topicCameraInfo = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;
  std::cout << "topic camera info: " << topicCameraInfo << std::endl;

  std::vector<std::string> topics;
  topics.push_back(topicColor);
  topics.push_back(topicDepth);
  topics.push_back(topicCameraInfo);

  rosbag::Bag bag;
  bag.open(fileName, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Time time;
  std::string type1, type2, type3;
  for (rosbag::View::iterator iter = view.begin(); iter != view.end(); ++iter)
  {
    rosbag::MessageInstance const m1 = *iter;
    ++iter;
    rosbag::MessageInstance const m2 = *iter;
    ++iter;
    rosbag::MessageInstance const m3 = *iter;

    time = m1.getTime();
    std::cout << "Read time: " << time << std::endl;

    type1 = m1.getDataType();
    type2 = m2.getDataType();
    type3 = m3.getDataType();

    std::cout << type1 << " " << type2 << " " << type3 << std::endl;

    // sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
    // if (image != NULL)
    // {
    //   if (image->encoding == "bgr8")
    //     std::cout << "Read color image" << std::endl;
    //   if (image->encoding == "16UC1")
    //     std::cout << "Read depth image" << std::endl;
    // }
    //
    //   sensor_msgs::CameraInfo::ConstPtr cameraInfo = m.instantiate<sensor_msgs::CameraInfo>();
    //   if (cameraInfo != NULL)
    //       std::cout << "Read camera info" << std::endl;

    std::cout << "Proceed? y/n" << std::endl;
    std::cin >> flag;

    if (flag == "n" || flag == "N")
      break;
  }

  bag.close();

  // clean shutdown
  ros::shutdown();
  return 0;
}
