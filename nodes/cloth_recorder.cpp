// Cloth recorder code to record various ros topics
// Depends on using rosbag api
// Author: Nishanth Koganti
// Date: 2015/7/26

// general headers
#include <iostream>

// ros headers
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// program variables
int count  = 0;
int limit = 100;

class ClothRecorder
{
  private:
    // variable for the rosbag
		rosbag::Bag bag;

    // rosnode handle parameter
    ros::NodeHandle n;

    // sensor message where the message will be stored
		sensor_msgs::PointCloud2Ptr x;

    // extracted point cloud subscriber
 		ros::Subscriber clothCloudSub;

    // rosbag fileName
    std::string fileName;

  // public class functions
  public:
    // class constructor
    ClothRecorder(ros::NodeHandle* n, std::string fileName)
    {
      // obtain ros node handle
      this->n = *n;

      // obtain the root file name for the ros handle
      this->fileName = fileName;

      // open a new bag file
		  bag.open(this->fileName, rosbag::bagmode::Write);

      // subscribe to the required ros topics
      this->clothCloudSub = this->n.subscribe("/cloth/points", 1, &ClothRecorder::record, this);
	  }

    // destructor function
    ~ClothRecorder()
    {

    }

    // callback function for the subscriber
		void record(const sensor_msgs::PointCloud2Ptr& msg)
		{
      // print out the count
			std::cout << count << std::endl;

      // write point cloud message to the ros bag file
      if(count < limit)
      {
        bag.write("/cloth/points", ros::Time::now(), *msg);
  			count++;
      }
      else
			{
				bag.close();
				clothCloudSub.shutdown();
        std::cout << "Saving ROS Bag" << std::endl;
			}
		}
};

int main(int argc, char *argv[])
{
  // initializing ros node
	ros::init(argc, argv, "ClothRecorder");

  // create ros node handle
  ros::NodeHandle n;

  // string name for the rosbag
	std::string bagName;

  // start the ClothRecorder class instance
  if (argc > 1)
  {
	  bagName = argv[1];
	  ClothRecorder recorder(&n,bagName);
	  ros::spin();
	}
	else
	{
		std::cout << "Usage: run program with fileName as input argument" << std::endl;
	}

  // clean ros shutdown
  ros::shutdown();
  return 0;
}
