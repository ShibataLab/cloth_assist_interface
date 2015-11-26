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
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/min_cut_segmentation.h>
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
  std::string fileName, calibName;
  char bagName[200], cloudName[200], colorName[200], depthName[200], maskName[200];

  // printing help information
  if(argc != 3)
  {
    help(argv[0]);
    ros::shutdown();
    return 0;
  }
  else
  {
    fileName = argv[1];
    calibName = argv[2];
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

  ifstream calibDat(calibName, ifstream::in);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  for (int i = 0; i < 4; i++)
    calibDat >> transform(i,0) >> c >> transform(i,1) >> c >> transform(i,2) >> c >> transform(i,3);
  std::cout << transform << std::endl;

  // variables for parsing ros bag file
  std::string topic, type;
  cv::Mat color, depth, mask;

  // pcl feature extraction Initialization
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSOR(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransform(new pcl::PointCloud<pcl::PointXYZ>());

  // feature instances
  pcl::PointXYZ cloudMean;
  Eigen::Vector4f centroid;
  std::vector<pcl::PointIndices> clusters;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);

  // Parameter setting for filtering
  sor.setMeanK(100);
  sor.setKeepOrganized(true);
  sor.setStddevMulThresh(0.7);

  // wait for key press
  // std::cin >> c;

  // pcl point cloud
  cv::namedWindow("Depth",CV_WINDOW_AUTOSIZE);
  pcl::visualization::CloudViewer viewer("Feature Extraction");

  // ros time init
  ros::Time::init();

  // ros iterator initialization
  int frame = 0;

  ros::Rate rate(30);
  ros::Duration tPass;
  ros::Time now, begin;
  std::vector<int> indices;
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

    sor.setInputCloud(cloud);
    sor.filter(*cloudSOR);

    pcl::transformPointCloud(*cloudSOR, *cloudTransform, transform);

    // Center point cloud
    pcl::compute3DCentroid(*cloudTransform, centroid);
    pcl::demeanPointCloud(*cloudTransform, centroid, *cloudCentered);

    for (int i = 0; i < cloudCentered->size(); i++)
    {
      if (!pcl_isfinite (cloudCentered->points[i].x) || !pcl_isfinite (cloudCentered->points[i].y) || !pcl_isfinite (cloudCentered->points[i].z))
      {
        cloudCentered->points[i].x = 0.0; cloudCentered->points[i].y = 0.0; cloudCentered->points[i].z = 0.0;
      }
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

    if (cloudCentered->size() != 62500)
      std::cout << "Error!: " << cloudCentered->size() << std::endl;

    for (int i = 0; i < cloudCentered->size()-1; i++)
    {
      cloudDat << cloudCentered->points[i].x << "," << cloudCentered->points[i].y << "," << cloudCentered->points[i].z << ",";
    }
    cloudDat << cloudCentered->points[cloudCentered->size()-1].x << "," << cloudCentered->points[cloudCentered->size()-1].y << "," << cloudCentered->points[cloudCentered->size()-1].z << std::endl;

    // std::cout << "Frame: " << frame << ", Time: " << tPass.toSec() << std::endl;
    // std::cout << "Cloud Centered: " << cloudCentered->size() << std::endl;
    frame++;

    viewer.showCloud(cloudCentered);
    cv::imshow("Depth", depth);
    rate.sleep();
  }

  bag.close();
  maskDat.close();
  colorDat.close();
  depthDat.close();
  cloudDat.close();

  std::cout << "Done!" << std::endl;
  
  // clean shutdown
  ros::shutdown();
  return 0;
}
