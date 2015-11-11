// kinect_processor.cpp: Program to read rosbag files and process kinect2 data
// Requirements: rosbag file as input
// Author: Nishanth Koganti
// Date: 2015/9/14

// header files
#include <processor.h>

// help function
void help(const std::string &path)
{
  std::cout << path << "  [options]" << std::endl
            << "  recordMode: 'qhd', 'hd', 'sd'" << std::endl
            << "  writeVideo: write color tracking video to file" << std::endl
            << "  writeCloud: write extracted point cloud to ros bag file" << std::endl;
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

  // default values of parameters for processor class
  bool videoMode = false;
  bool cloudMode = false;
  std::string topicType = "qhd";
  std::string ns = K2_DEFAULT_NS;
  std::string fileName = "default";
  std::string topicColor, topicDepth, topicCameraInfo;
  topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

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

    // writeVideo mode
    else if(param == "writeVideo")
      videoMode = true;

    // writeCloud mode
    else if(param == "writeCloud")
      cloudMode = true;

    // other option can only be the fileName
    else
      fileName = param;
  }

  // initializing color and depth topic names with kinect2 header
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  topicCameraInfo = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

  // create instance of processor class
  Processor processor(fileName, topicColor, topicDepth, topicCameraInfo, topicType, videoMode, cloudMode);

  // run the processor class to parse rosbag file and get results
  processor.run();

  // clean shutdown
  ros::shutdown();
  return 0;
}
