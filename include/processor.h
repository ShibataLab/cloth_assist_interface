// processor.h: class definition for processing point cloud data
// Requirements: rosbag file as input
// Author: Nishanth Koganti
// Date: 2015/9/7

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

// ns = kinect2
// selecting default topic names when the options are not provided
std::string topicType = "qhd";
std::string ns = K2_DEFAULT_NS;
std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

std::vector<std::string> topics;
topics.push_back(topicColor);
topics.push_back(topicDepth);
topics.push_back(topicCameraInfo);

rosbag::Bag bag;
bag.open(fileName, rosbag::bagmode::Read);
rosbag::View view(bag, rosbag::TopicQuery(topics));

// tracker.h: Tracker class definition for tracking clothing articles
// Requirements: OpenCV and PCL headers and libraries installed
// Author: Nishanth Koganti
// Date: 2015/9/4

// TODO:

#pragma once
#ifndef __TRACKER_H__
#define __TRACKER_H__

// CPP headers
#include <string>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <iostream>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// ROS headers
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <sensor_msgs/Image.h>

// Preprocessor directives
#define SMIN 30
#define VMIN 30
#define VMAX 250

// Tracker class
class Tracker
{
  public:
    // pcl initialization
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;

    Tracker();

    // class destructor
    ~Tracker();

    // setters
    void setImages(cv::Mat &color, cv::Mat &depth, ros::Time time);

    // function to obtain cloth calibration values
    void clothCalibrate(cv::Mat &color, cv::Mat &depth);

    // getters
    void getOutput(cv::Mat &output, cv::Mat &backproj);

  private:
    // rosbag variables
    ros::Time m_time;
    rosbag::Bag cloudBag;

    // opencv parameters
    cv::Point m_origin;
    bool m_selectObject;
    int m_width, m_height;
    cv::Rect m_selection, m_window;
    cv::Mat m_color, m_depth, m_hist, m_output, m_backproj;

    // mouse click callback function for T-shirt color calibration
    static void onMouse(int event, int x, int y, int flags, void* param);

    // function to display images
    void cloudExtract();

    // function to get ROI from color and depth images
    void createROI(cv::Mat &roi);

    // function to build point cloud from roi
    void createCloud(cv::Mat &roi);
};

#endif
