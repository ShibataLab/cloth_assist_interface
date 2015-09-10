// kinect_extract.cpp: Program to extract different features from constructed point cloud
// Requirements: rosbag file as input and pcl libraries
// Author: Nishanth Koganti
// Date: 2015/9/2

// Test comment

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

// PCL headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

// help function
void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  fileName: name of rosbag file" << std::endl;
}

int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "kinect_extract", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // filename default
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

    // other option can only be the fileName
    else
      fileName = param;
  }

  // initializing color and depth topic names
  std::string topicCloud = "/cloth/cloud";
  std::cout << "topic cloud: " << topicCloud << std::endl;

  std::vector<std::string> topics;
  topics.push_back(topicCloud);

  rosbag::Bag bag;
  bag.open(fileName, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // pcl point cloud
  pcl::visualization::CloudViewer viewer("Feature Extraction");

  // pcl feature extraction Initialization
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVOG(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSOR(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCentered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr neTree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr vfhTree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
  pcl::PointCloud<pcl::ESFSignature640>::Ptr esfs(new pcl::PointCloud<pcl::ESFSignature640>());


  // feature instances
  Eigen::Vector4f centroid;
  pcl::VoxelGrid<pcl::PointXYZ> vog;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;

  // Parameter setting for filtering
  vog.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.setMeanK(100);
  sor.setStddevMulThresh(0.7);
  vfh.setSearchMethod(vfhTree);
  ne.setRadiusSearch(0.03);
  ne.setSearchMethod(neTree);

  // ros time init
  ros::Time::init();

  // ros iterator initialization
  int frame = 0;
  ros::Time time;
  ros::Rate rate(30);
  rosbag::View::iterator iter = view.begin();

  while(iter != view.end())
  {
    time = (*iter).getTime();

    rosbag::MessageInstance const m = *iter;

    cloud = m.instantiate<pcl::PointCloud <pcl::PointXYZ> >();
    ++iter;

    vog.setInputCloud(cloud);
    vog.filter(*cloudVOG);

    sor.setInputCloud(cloudVOG);
    sor.filter(*cloudSOR);

    // Center point cloud
    compute3DCentroid(*cloudSOR,centroid);
    demeanPointCloud(*cloudSOR, centroid,*cloudCentered);

    // Normal Estimation
    ne.setInputCloud(cloudCentered);
    ne.compute(*cloudNormals);

    // Viewpoint Feature Histogram
    vfh.setInputCloud(cloudCentered);
    vfh.setInputNormals(cloudNormals);
    vfh.compute(*vfhs);

    // Ensemble of Shape Functions
    esf.setInputCloud(cloudCentered);
    esf.compute(*esfs);

    viewer.showCloud(cloudCentered);

    std::cout << "Frame: " << frame << ", Time: " << time << std::endl;
    frame++;

    rate.sleep();
  }

  bag.close();

  // clean shutdown
  ros::shutdown();
  return 0;
}
