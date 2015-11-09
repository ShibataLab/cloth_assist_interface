// cloth_tracker.cpp: Program to track clothing articles
// Requirements: depends on tracker class files and iai kinect2 bridge
// Author: Nishanth Koganti
// Date: 2015/10/27

// TODO:

// tracker class definition
# include <tracker.h>

// help function
void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  mode: 'qhd', 'hd', 'sd'" << std::endl;
}

// main function
int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "cloth_tracker", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // argument for calibration file
  std::string calibFile = "default";

  // create default strings for different kinect2 ros topics
  std::string topicType = "sd";
  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

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

    // setting the calib file parameter
    else
    {
      calibFile = param;
    }
  }


  // initializing color and depth topic names
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  // create tracker with parsed command line parameters
  Tracker tracker(topicColor, topicDepth, topicType, calibFile);

  // start running tracker instance
  std::cout << "starting tracker..." << std::endl;
  tracker.run();

  // clean shutdown
  ros::shutdown();
  return 0;
}
