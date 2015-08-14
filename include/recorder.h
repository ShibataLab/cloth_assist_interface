// Recorder Class Definition
// Class depends on the IAI Kinect2 Package
// Author: Nishanth Koganti
// Date: 2015/7/26

#pragma once
#ifndef __CLOTH_RECORDER_H__
#define __CLOTH_RECORDER_H__

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

// ZeroMQ header
#include <zmq.hpp>

// ROS headers
#include <ros/ros.h>
#include <rosbag/bag.h>
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

/******************************************************************************/
// Recorder Class
class Recorder
{
  private:
    // important variables to decide functionality
    const std::string topicColor, topicDepth;
    std::string fileName, topicCameraInfoColor, topicCameraInfoDepth;

    // flags for logic
    size_t frame;
    const size_t queueSize;

    // variable for the rosbag
    rosbag::Bag bag;

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

  public:
    // class constructor
    // takes 3 input arguments topic color, topic depth and topic type
    Recorder(const std::string &topicColor, const std::string &topicDepth);

    // class destructor
    ~Recorder();

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
};

#endif
