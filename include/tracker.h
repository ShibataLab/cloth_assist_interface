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
#include <pcl/features/esf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    std::mutex _lock;

    // important variables to decide functionality
    const std::string _topicColor, _topicDepth, _topicType, _calibFile;
    std::string _topicCameraInfoColor, _topicCameraInfoDepth;

    // flags for logic
    const size_t _queueSize;
    bool _running, _updateImage;

    // opencv parameters
    cv::Point _origin;
    int _width, _height;
    cv::Mat _lookupX, _lookupY;
    bool _selectObject = false;
    cv::Mat _color, _depth, _hist;
    cv::Rect _selection, _window;
    cv::Mat _cameraMatrixColor, _cameraMatrixDepth;

    // synchronization parameters
    // message filters are used to take a message and output it at a different time
    // it is usually used to synchronize different ros topics
    // URL: http://wiki.ros.org/message_filters

    // filters are being used to synchronize 2 image and 2 cameraInfo topics i.e. 4 in total
    // here the typedefs for later use are being defined
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> _ExactSyncPolicy;

    // ros node parameters
    ros::NodeHandle _nh;

    // AsyncSpinner is used to synchronize multiple threads under the ros framework
    // URL:http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning#Multi-threaded_Spinning
    ros::AsyncSpinner _spinner;

    // image transport should be used when publishing or subscribing to image topics
    // URL: http://wiki.ros.org/image_transport
    image_transport::ImageTransport _it;

    // SubscriberFilter is a image transport subscriber that interfaces with message filters
    // it directly sends the image to message filters for further processing.
    // URL: http://docs.ros.org/jade/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html
    image_transport::SubscriberFilter *_subImageColor, *_subImageDepth;

    // message filters
    message_filters::Synchronizer<_ExactSyncPolicy> *_syncExact;

    // creating message filters for the camera info topics
    message_filters::Subscriber<sensor_msgs::CameraInfo> *_subCameraInfoColor, *_subCameraInfoDepth;

    // create publisher for publishing ros topics as well
    ros::Publisher _pubPointCloud;
    ros::Publisher _pubESFDescriptor;
    image_transport::Publisher _pubTrackImage;

    // pcl feature extraction Initialization
    Eigen::Matrix4f _transform;
    pcl::PointCloud<pcl::ESFSignature640>::Ptr _cloudESF;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, _cloudVOG, _cloudSOR, _cloudTransform, _cloudCentered;

    // feature instances
    Eigen::Vector4f _centroid;
    pcl::VoxelGrid<pcl::PointXYZ> _vog;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> _sor;
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> _esf;

  public:
    // class constructor
    // takes 3 input arguments topic color, topic depth and topic type
    Tracker(const std::string &topicColor, const std::string &topicDepth, const std::string &topicType, const std::string &calibFile);

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
    void createCloud(cv::Mat &roi);

    // function to create processor for cloud
    void processCloud();

    // function to create lookup table for obtaining x,y,z values
    void createLookup(size_t width, size_t height);
};

#endif
