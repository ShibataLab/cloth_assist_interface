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

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// Boost headers
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// ROS messaging filters
#include <message_filters/subscriber.h>

// kinect2 bridge header
#include <kinect2_bridge/kinect2_definitions.h>

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
  // obtain image data and encoding from sensor msg
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);

  // copy data to the Mat image
  pCvImage->image.copyTo(image);
}

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

  // opencv variables
  cv::Mat color, depth;
  cv::namedWindow("Color",0);
  cv::namedWindow("Depth",0);

  int frame = 0;
  ros::Time time;
  std::string type;
  rosbag::View::iterator iter = view.begin();

  while(iter != view.end())
  {
    time = (*iter).getTime();

    for (int i = 0; i < 3; i++)
    {
      rosbag::MessageInstance const m = *iter;

      type = m.getDataType();
      if (type == "sensor_msgs/Image")
      {
        sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
        if (image->encoding == "bgr8")
          readImage(image, color);
        else if (image->encoding == "16UC1")
          readImage(image, depth);
      }
      else
        sensor_msgs::CameraInfo::ConstPtr cameraInfo = m.instantiate<sensor_msgs::CameraInfo>();

      ++iter;
    }

    cv::imshow("Color", color);
    cv::imshow("Depth", depth);
    cv::waitKey(30);

    std::cout << "Frame: " << frame << ", Time: " << time << std::endl;
    frame++;
  }

  bag.close();

  // clean shutdown
  ros::shutdown();
  return 0;
}