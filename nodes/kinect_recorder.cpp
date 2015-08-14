// Kinect Topics Recorder using ros bag
// Depends on IAI Kinect2 Bridge and ROS Bag
// Author: Nishanth Koganti
// Date: 2015/8/1

# include <recorder.h>

/******************************************************************************/
// help function
void help(const std::string &path)
{
  std::cout << path << " [options]" << std::endl
            << "  mode: 'qhd', 'hd', 'sd'" << std::endl;
}

/******************************************************************************/
// main function
int main(int argc, char **argv)
{
  // create ros node with a random name to avoid conflicts
  ros::init(argc, argv, "kinect_recorder", ros::init_options::AnonymousName);

  // check for failure
  if(!ros::ok())
  {
    return 0;
  }

  // create strings for different kinect2 ros topics
  // ns = kinect2
  std::string topicType = "sd";
  std::string ns = K2_DEFAULT_NS;
  std::string fileName = "kinect";
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
  }

  // initializing color and depth topic names
  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  std::cout << "topic color: " << topicColor << std::endl;
  std::cout << "topic depth: " << topicDepth << std::endl;

  // create processor with parsed command line parameters
  Recorder recorder(topicColor, topicDepth);

  // start running processor instance
  std::cout << "starting recorder..." << std::endl;
  recorder.run();

  // clean shutdown
  ros::shutdown();
  return 0;
}
