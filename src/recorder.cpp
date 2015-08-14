// Recorder Class Definition
// Relies on the IAI Kinect2 Bridge and ROS Bag
// Author: Nishanth Koganti
// Date: 2015/8/1

#include <recorder.h>

// class constructor
Recorder::Recorder(const std::string &topicColor, const std::string &topicDepth)
    : topicColor(topicColor), topicDepth(topicDepth), frame(0), queueSize(5), nh("~"), spinner(0), it(nh)
{
}

// class destructor
Recorder::~Recorder()
{
}

// run function
void Recorder::run()
{
  // zmq variables
  std::string Message;

  // start the kinect depth sensor
  start();

  // initialize zmq variables
  zmq::context_t context(1);
  zmq::socket_t kinectSocket(context, ZMQ_PAIR);

  kinectSocket.bind("tcp://*:5555");
  std::cout << "[ZMQ] Created kinect server" << std::endl;

  while (1)
  	{
  		// Get the file name
  		zmq::message_t request;
  		kinectSocket.recv(&request);
  		Message = std::string(static_cast<char *>(request.data()), request.size());

      if (Message == "StopServer")
  		{
  			std::cout << "[ZMQ] Received Stop Server" << std::endl;
  			break;
  		}

      else if (Message == "NewTrial")
  		{
  			std::cout << "[ZMQ] Received NewTrial" << std::endl;

  			// send ready reply
  			zmq::message_t reply(5);
  			std::memcpy((void *)reply.data(), "Ready", 5);
  			kinectSocket.send(reply);
  			std::cout << "[ZMQ] Sent Ready" << std::endl;

  			// get kinect filename
  			zmq::message_t recordName;
  			kinectSocket.recv(&recordName);
  			fileName = std::string(static_cast<char *>(recordName.data()), recordName.size());
  			std::cout << "[ZMQ] Received filename" << std::endl;

        // open a new bag file
        bag.open(fileName, rosbag::bagmode::Write);
        std::cout << "[ROSBAG] Opening ROS bag" << std::endl;

        zmq::message_t startRequest;
  			kinectSocket.recv(&startRequest);
  			Message = std::string(static_cast<char *>(startRequest.data()), startRequest.size());
        if (Message == "StartRecording")
        {
          std::cout << "[ZMQ] Received StartRecording" << std::endl;

          // start the recording
          spinner.start();
        }
  			else
        {
          std::cout << "[ZMQ] Invalid message" << std::endl;
          continue;
        }

        // Get the file name
    		zmq::message_t stopRequest;
    		kinectSocket.recv(&stopRequest);
    		Message = std::string(static_cast<char *>(stopRequest.data()), stopRequest.size());
        if (Message == "StopRecording")
        {
          std::cout << "[ZMQ] Received StopRecording" << std::endl;
        }
  			else
        {
          std::cout << "[ZMQ] Invalid message" << std::endl;
        }

        // stop the spinner
        spinner.stop();

        // close rosbag
        bag.close();
        std::cout << "[ROSBAG] Saving ROS Bag" << std::endl;
  		}
  	}

  // clean shutdown
  kinectSocket.close();
  stop();
}

// start function
void Recorder::start()
{
  // get the camera info ros topics for color and depth
  topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
  topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";


  // TransportHints stores the transport settings for the image topic subscriber
  // here we are giving the setting of raw or compressed using the useCompressed variable
  image_transport::TransportHints hints("raw");

  // SubscriberFilters are used to subscribe to the kinect image topics
  subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
  subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);

  // message filters are used to subscribe to the camera info topics
  subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
  subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

  // creating a exact synchronizer for 4 ros topics with queueSize
  // the recorder class callback function is set as the callback function
  syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
  syncExact->registerCallback(boost::bind(&Recorder::callback, this, _1, _2, _3, _4));
}

// stop function to have clean shutdown
void Recorder::stop()
{
  // clean up all variables
  delete syncExact;

  delete subImageColor;
  delete subImageDepth;
  delete subCameraInfoColor;
  delete subCameraInfoDepth;
}

// message filter callback function
void Recorder::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
  ros::Time time = ros::Time::now();

  // write point cloud message to the ros bag file
  bag.write(topicColor, time, *imageColor);
  bag.write(topicDepth, time, *imageDepth);
  bag.write(topicCameraInfoColor, time, *cameraInfoColor);
}
