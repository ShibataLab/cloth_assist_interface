// feature_extract.cpp: Program to extract different features from features ros bag
// Requirements: rosbag file as input and pcl libraries
// Author: Nishanth Koganti
// Date: 2015/11/16

// CPP headers
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
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

// help function
void help(const std::string &path)
{
  std::cout << path << "  [options]" << std::endl
            << "  fileName: name of rosbag file" << std::endl;
}

// function to get cv::Mat from sensor_msgs::Image
void readImage(sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
  // obtain image data and encoding from sensor msg
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);

  // copy data to the Mat image
  pCvImage->image.copyTo(image);
}

int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "feature_extract", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // filename default
  char c;
  std::string fileName;
  char bagName[200], cloudName[200], colorName[200], depthName[200], maskName[200];

  // printing help information
  if(argc != 2)
  {
    help(argv[0]);
    ros::shutdown();
    return 0;
  }
  else
  {
    fileName = argv[1];
  }

  // create fileNames for different output files
  sprintf(maskName, "%sMask", fileName.c_str());
  sprintf(bagName, "%sCloud.bag", fileName.c_str());
  sprintf(cloudName, "%sCloud", fileName.c_str());
  sprintf(colorName, "%sColor", fileName.c_str());
  sprintf(depthName, "%sDepth", fileName.c_str());

  ofstream maskDat(maskName, ofstream::out);
  ofstream colorDat(colorName, ofstream::out);
  ofstream depthDat(depthName, ofstream::out);
	ofstream cloudDat(cloudName, ofstream::out);

  // initializing the topic names
  std::string topicCloud = "/cloth/cloud";
  std::string topicColor = "/cloth/color";
  std::string topicDepth = "/cloth/depth";
  std::string topicMask = "/cloth/backproj";

  std::vector<std::string> topics;
  topics.push_back(topicDepth); topics.push_back(topicMask);
  topics.push_back(topicCloud); topics.push_back(topicColor);

  rosbag::Bag bag;
  bag.open(bagName, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // variables for parsing ros bag file
  std::string topic, type;
  cv::namedWindow("Color",1);
  cv::Mat color, depth, mask;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  // wait for key press
  // std::cin >> c;

  // ros time init
  ros::Time::init();

  // ros iterator initialization
  int frame = 0;

  ros::Rate rate(30);
  ros::Duration tPass;
  ros::Time now, begin;
  rosbag::View::iterator iter = view.begin();

  while(iter != view.end())
  {
    now = (*iter).getTime();
    if (frame == 0)
      begin = (*iter).getTime();
    tPass = now - begin;

    for (int i = 0; i < 4; i++)
    {
      rosbag::MessageInstance const m = *iter;
      type = m.getDataType(); topic = m.getTopic();

      if (type == "sensor_msgs/Image")
      {
        sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
        if (topic == "/cloth/backproj")
          readImage(image, mask);
        else if (topic == "/cloth/color")
          readImage(image, color);
        else
          readImage(image, depth);
      }
      else
        cloud = m.instantiate<pcl::PointCloud <pcl::PointXYZ> >();
      ++iter;
    }

    for(int r = 0; r < color.rows; ++r)
    {
      const uint8_t *itMask = mask.ptr<uint8_t>(r);
      const uint8_t *itColor = color.ptr<uint8_t>(r);
      const uint8_t *itDepth = depth.ptr<uint8_t>(r);

      for(int c = 0; c < color.cols; ++c, ++itColor, ++itDepth, ++itMask)
      {
        maskDat << (int) *itMask << ",";
        colorDat << (int) *itColor << ",";
        depthDat << (int) *itDepth << ",";
      }
    }
    maskDat << std::endl; colorDat << std::endl; depthDat << std::endl;

    for (int i = 0; i < cloud->size()-1; i++)
      cloudDat << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << ",";
    cloudDat << cloud->points[cloud->size()-1].x << "," << cloud->points[cloud->size()-1].y << "," << cloud->points[cloud->size()-1].z << std::endl;

    std::cout << "Frame: " << frame << ", Time: " << tPass.toSec() << std::endl;
    frame++;

    cv::imshow("Color", color);
    rate.sleep();
  }

  bag.close();
  maskDat.close();
  colorDat.close();
  depthDat.close();
  cloudDat.close();

  // clean shutdown
  ros::shutdown();
  return 0;
}
