// tracker.h: Tracker class definition for tracking clothing articles
// Requirements: OpenCV and PCL headers and libraries installed with iai kinect2 repo
// Author: Nishanth Koganti
// Date: 2015/8/22

// TODO:

#pragma once
#ifndef __CLOTH_TRACKER_H__
#define __CLOTH_TRACKER_H__

// CPP headers
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
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
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// ROS messaging filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// kinect2 bridge header
#include <kinect2_bridge/kinect2_definitions.h>

// Preprocessor directives
#define SMIN 20
#define VMIN 10
#define VMAX 256

// Tracker class
class Tracker
{
  private:
    // mutex for synchronization
    std::mutex lock;

    // important variables to decide functionality
    const std::string topicColor, topicDepth, topicType;

    // flags for logic
    size_t frame;
    const size_t queueSize;
    bool running, updateImage;

    // opencv parameters
    cv::Point origin;
    int width, height;
    cv::Mat lookupX, lookupY;
    bool selectObject = false;
    cv::Mat color, depth, hist;
    cv::Rect selection, window;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;

    // synchronization parameters
    // message filters are used to take a message and output it at a different time
    // it is usually used to synchronize different ros topics
    // URL: http://wiki.ros.org/message_filters

    // filters are being used to synchronize 2 image and 2 cameraInfo topics i.e. 4 in total
    // here the typedefs for later use are being defined
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;

    // ros node parameters
    ros::NodeHandle nh;

    // AsyncSpinner is used to synchronize multiple threads under the ros framework
    // URL:http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
    ros::AsyncSpinner spinner;

    // image transport should be used when publishing or subscribing to image topics
    // URL: http://wiki.ros.org/image_transport
    image_transport::ImageTransport it;

    // SubscriberFilter is a image transport subscriber that interfaces with message filters
    // it directly sends the image to message filters for further processing.
    // URL: http://docs.ros.org/jade/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html
    image_transport::SubscriberFilter *subImageColor, *subImageDepth;

    // message filters
    message_filters::Synchronizer<ExactSyncPolicy> *syncExact;

    // creating message filters for the camera info topics
    message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

    // create publisher for publishing ros topics as well
    ros::Publisher pubPointCloud;

    // std variables
    std::ostringstream oss;
    std::vector<int> params;

  public:
    // class constructor
    // takes 3 input arguments topic color, topic depth and topic type
    Tracker(const std::string &topicColor, const std::string &topicDepth, const std::string &topicType);

    // class destructor
    ~Tracker();

    // run function
    void run();

  private:
    // start function
    void start();

    // stop function to have clean shutdown
    void stop();

    // message filter callback function
    void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
      const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth);

    // mouse click callback function for T-shirt color calibration
    static void onMouse(int event, int x, int y, int flags, void* param);

    // function to obtain cloth calibration values
    void clothCalibrate();

    // function to display images
    void clothTracker();

    // function to obtain Mat from ros image sensor_msg
    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const;

    // function to obtain camera info from message filter msgs
    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const;

    // function to get ROI from color and depth images
    void createROI(cv::Mat &color, cv::Mat &depth, cv::Mat &backproj, cv::Mat &roi);

    // function to build point cloud from roi
    void createCloud(cv::Mat &roi, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // function to create lookup table for obtaining x,y,z values
    void createLookup(size_t width, size_t height);
};

#endif
