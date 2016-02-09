// kinect_recorder.cpp: Program to subscribe to kinect2 topics and synchronize them
// to save to a ros bag file.
// Requirements: recorder class files and synchronizer client running for control
// Author: Nishanth Koganti
// Date: 2015/8/22

// all headers recorded in class definition header
# include <recorder.h>

// help function
void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  mode: 'qhd', 'hd', 'sd'" << std::endl;
}

int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "kinect_recorder", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // ns = kinect2
  // selecting default topic names when the options are not provided
  int recordMode = 0;
  std::string topicType = "qhd";
  std::string ns = K2_DEFAULT_NS;
  std::string fileName = "kinect";
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

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

    // remaining parameter is record mode
    else
    {
      std::istringstream iss(param);
      iss >> recordMode;
    }
  }

  // initializing color and depth topic names
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  // create processor with parsed command line parameters
  Recorder recorder(topicColor, topicDepth, recordMode);

  // start running processor instance
  std::cout << "starting recorder..." << std::endl;
  recorder.run();

  // clean shutdown
  ros::shutdown();
  return 0;
}
