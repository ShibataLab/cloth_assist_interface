// tracker.cpp: Tracker class function declaration for tracking clothing articles
// using the cam shift algorithm and obtain point cloud using pcl functions
// Requirements: relies on the use of iai kinect2 bridge, opencv and pcl
// Author: Nishanth Koganti
// Date: 2015/8/22

// TODO:
// 1) Improve point cloud processing using different filters

#include <tracker.h>

// class constructor
Tracker::Tracker(const std::string &topicColor, const std::string &topicDepth, const std::string &topicType)
    : topicColor(topicColor), topicDepth(topicDepth), topicType(topicType), updateImage(false), running(false), frame(0), queueSize(5), nh("~"), spinner(0), it(nh)
{
  // create matrices for intrinsic parameters
  cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
  cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
}

// class destructor
Tracker::~Tracker()
{
}

// run function
void Tracker::run()
{
  // start the kinect depth sensor
  start();

  // perform cloth calibration
  clothCalibrate();

  // start image viewer
  clothTracker();

  // clean shutdown
  stop();
}

// start function
void Tracker::start()
{
  // set running flag, will be unset on shutdown
  running = true;

  // get the camera info ros topics for color and depth
  std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
  std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

  // TransportHints stores the transport settings for the image topic subscriber
  // here we are giving the setting of raw or compressed using the useCompressed variable
  image_transport::TransportHints hints("raw");

  // SubscriberFilters are used to subscribe to the kinect image topics
  subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
  subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);

  // message filters are used to subscribe to the camera info topics
  subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
  subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

  // create ros publisher
  pubPointCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloth/points", 5);

  // creating a exact synchronizer for 4 ros topics with queueSize
  // the Tracker class callback function is set as the callback function
  syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
  syncExact->registerCallback(boost::bind(&Tracker::callback, this, _1, _2, _3, _4));

  // set the width and height parameters for cloth tracking functions
  if(topicType == "hd")
  {
    this->height = 1080;
    this->width = 1920;
  }
  else if(topicType == "qhd")
  {
    this->height = 540;
    this->width = 960;
  }
  else if(topicType == "sd")
  {
    this->height = 424;
    this->width = 512;
  }

  // from here we start the 4 threads
  spinner.start();

  // create a chrono instance for 1 millisecond
  std::chrono::milliseconds duration(1);

  // if the updateImage flags are not set then we will check for ros shutdown
  while(!updateImage)
  {
    if(!ros::ok())
    {
      return;
    }
    std::this_thread::sleep_for(duration);
  }
}

// stop function to have clean shutdown
void Tracker::stop()
{
  // stop the spinner
  spinner.stop();

  // clean up all variables
  delete syncExact;

  delete subImageColor;
  delete subImageDepth;
  delete subCameraInfoColor;
  delete subCameraInfoDepth;

  running = false;
}

// message filter callback function
void Tracker::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
  // initialization
  cv::Mat color, depth;

  // get the camera info and images
  readCameraInfo(cameraInfoColor, cameraMatrixColor);
  readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
  readImage(imageColor, color);
  readImage(imageDepth, depth);

  // IR image input
  if(color.type() == CV_16U)
  {
    cv::Mat tmp;
    color.convertTo(tmp, CV_8U, 0.02);
    cv::cvtColor(tmp, color, CV_GRAY2BGR);
  }

  // apply mutex and then save the data to class variables
  lock.lock();
  this->color = color;
  this->depth = depth;
  updateImage = true;
  lock.unlock();
}

// function to obtain cloth calibration values
// cloth calibrate function
void Tracker::clothCalibrate()
{
	// opencv initialization
	int key = 0;
	cv::Mat color, depth, disp, hsv, mask, hue;

	// GUI initialization
	cv::namedWindow("CamShift", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("CamShift", onMouse, (void *)this);

	// get color and depth images
  for(; running && ros::ok();)
  {
    if(updateImage)
    {
      // lock the class variables
      lock.lock();
      color = this->color;
      depth = this->depth;
      updateImage = false;
      lock.unlock();
      break;
    }
  }

  color.copyTo(disp);

  // inifinite for loop
  for(; running && ros::ok();)
  {
		// show the selection
		if (this->selectObject && this->selection.width > 0 && this->selection.height > 0)
		{
			color.copyTo(disp);
			cv::Mat roi(disp, selection);
			cv::bitwise_not(roi, roi);
		}

		// display the image
		cv::imshow("CamShift", disp);
		key = cv::waitKey(5);
		if (key == 'q')
			break;
	}

	// destroy the window
	cv::destroyWindow("CamShift");

	// convert image to hsv space
	cv::cvtColor(color, hsv, CV_BGR2HSV);

	// perform HSV based color extraction
	cv::inRange(hsv, cv::Scalar(0, SMIN, VMIN), cv::Scalar(180, 256, VMAX), mask);

	// get the hue channel
	int ch[] = { 0, 0 };
	hue.create(hsv.size(), hsv.depth());

	// get the hue channel
	mixChannels(&hsv, 1, &hue, 1, ch, 1);

	// parameter initialization for computing the histograms
	int hsize = 16;
	float hranges[] = { 0, 180 };
	const float* phranges = hranges;

	// get the ROIs from the mask, hue images
	cv::Mat roi(hue, this->selection), maskroi(mask, this->selection);

	// compute the histogram and normaize the histogram
	cv::calcHist(&roi, 1, 0, maskroi, this->hist, 1, &hsize, &phranges);
	cv::normalize(this->hist, this->hist, 0, 255, cv::NORM_MINMAX);

	// track window initialization
	this->window = this->selection;
}

// function to display images
void Tracker::clothTracker()
{
  // variable initialization
  cv::Rect window;
  cv::Mat color, depth, roi, backproj;

  // create chrono instances to measure performance of the program
  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;

  // more variables for printing text and other functions
  double fps = 0;
  size_t frameCount = 0;
  std::ostringstream oss;

  // create named windows for displaying color and backprojection images
  cv::namedWindow("Color");
  cv::namedWindow("Backproj");

  // pcl initialization
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::visualization::CloudViewer viewer("Cloth Point Cloud");

  // obtain starting time point
  start = std::chrono::high_resolution_clock::now();

  // inifinite for loop
  for(; running && ros::ok();)
  {
    // check for image update
    if(updateImage)
    {
      // lock the class variables
      lock.lock();
      color = this->color;
      depth = this->depth;
      updateImage = false;
      lock.unlock();

      ++frameCount;

      // get the time elapsed so far
      now = std::chrono::high_resolution_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;

      if(elapsed >= 1.0)
      {
        fps = frameCount / elapsed;
        oss.str("");
        oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
        start = now;
        frameCount = 0;
      }

      // function to extract image roi from color and depth functions
      createROI(color, depth, backproj, roi);

      // initialize cloud
      cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

      // set cloud parameters
      cloud->header.frame_id = "cloth_frame";
      cloud->width = roi.cols;
      cloud->height = roi.rows;

      cloud->is_dense = false;
      cloud->points.resize(cloud->height * cloud->width);

      // create lookup tables for x and y mappings
      createLookup(this->width, this->height);

      // function to create point cloud and obtain
      createCloud(roi, cloud);

      // publish the point cloud message
      pubPointCloud.publish(cloud);

      // show image
      cv::imshow("Color", color);
      cv::imshow("Backproj", backproj);

      // display point cloud
      if (cloud->size() > 0)
        viewer.showCloud(cloud);
    }

    // wait for key press
    int key = cv::waitKey(1);

    // check for different key presses
    switch(key & 0xFF)
    {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
        break;
    }
  }

  // destroy all windows and shutdown
  cv::destroyAllWindows();
  cv::waitKey(100);
}

// function to get image roi from color and depth images
void Tracker::createROI(cv::Mat &color, cv::Mat &depth, cv::Mat &backproj, cv::Mat &roi)
{
  // cam shift initialization
  cv::Rect window;
  int ch[] = { 0, 0 };
  cv::RotatedRect box;
  float hranges[] = { 0, 180 };
  const float* phranges = hranges;
  cv::Mat hsv, hue, mask, hist, pcImg, pcMask;

  // variables for text
  const int lineText = 1;
  const cv::Point pos(5, 15);
  const double sizeText = 0.5;
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const cv::Scalar colorText = CV_RGB(255, 255, 255);

  // initialize the calibration values
  hist = this->hist;
  window = this->window;

  // perform color extraction
  cv::cvtColor(color, hsv, CV_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, SMIN, VMIN), cv::Scalar(180, 256, VMAX), mask);

  // getting the hue channel
  hue.create(hsv.size(), hsv.depth());
  cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

  // calculating the back projection
  cv::calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
  backproj &= mask;

  // eroding and dilating to remove noise
  cv::dilate(backproj, backproj, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  cv::erode(backproj, backproj, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

  // applying camshift algorithm
  box = cv::CamShift(backproj, window, cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1));

  // reinitialize track window
  if (window.area() <= 1)
    window = cv::Rect(0, 0, this->width, this->height);
  this->window = window;

  window = cv::Rect(window.x - 15, window.y - 15, window.width + 15, window.height + 15);

  // draw rectangles around highlighted areas
  cv::rectangle(color, window.tl(), window.br(), cv::Scalar(255, 255, 255), 2, CV_AA);

  // put text in the image
  cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
  cv::putText(backproj, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);

  roi.release();

  pcImg = depth(window);
  pcMask = backproj(window);
  pcImg.copyTo(roi, pcMask);
}

// function to create point cloud from extracted ROI
void Tracker::createCloud(cv::Mat &roi, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  // variables
  cv::Rect window = this->window;
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  // parallel processing of pixel values
  #pragma omp parallel for
  for(int r = 0; r < roi.rows; ++r)
  {
    // create row of Points
    pcl::PointXYZ *itP = &cloud->points[r * roi.cols];

    // get pointer to row in depth image
    const uint16_t *itD = roi.ptr<uint16_t>(r);

    // get the x and y values
    const float y = lookupY.at<float>(0, window.y+r);
    const float *itX = lookupX.ptr<float>();
    itX = itX + window.x;

    // convert all the depth values in the depth image to Points in point cloud
    for(size_t c = 0; c < (size_t)roi.cols; ++c, ++itP, ++itD, ++itX)
    {
      register const float depthValue = *itD / 1000.0f;

      // Check for invalid measurements
      if(isnan(depthValue) || depthValue <= 0.001)
      {
        // set values to NaN for later processing
        itP->x = itP->y = itP->z = badPoint;
        continue;
      }

      // set the values for good points
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
    }
  }
}

// mouse click callback function for T-shirt color calibration
void Tracker::onMouse(int event, int x, int y, int flags, void* param)
{
  // this line needs to be added if we want to access the class private parameters
  // within a static function
  // URL: http://stackoverflow.com/questions/14062501/giving-callback-function-access-to-class-data-members-in-c
  Tracker* ptr = (Tracker*) param;

	if (ptr->selectObject)
	{
		ptr->selection.x = std::min(x, ptr->origin.x);
		ptr->selection.y = std::min(y, ptr->origin.y);
		ptr->selection.width = std::abs(x - ptr->origin.x);
		ptr->selection.height = std::abs(y - ptr->origin.y);
		ptr->selection &= cv::Rect(0, 0, ptr->width, ptr->height);
	}

	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		ptr->origin = cv::Point(x, y);
		ptr->selection = cv::Rect(x, y, 0, 0);
		ptr->selectObject = true;
		break;
	case cv::EVENT_LBUTTONUP:
		ptr->selectObject = false;
		break;
	}
}

// function to obtain Mat from ros image sensor_msg
void Tracker::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
{
  // obtain image data and encoding from sensor msg
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);

  // copy data to the Mat image
  pCvImage->image.copyTo(image);
}

// function to obtain camera info from message filter msgs
void Tracker::readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
{
  // get pointer for first element in cameraMatrix
  double *itC = cameraMatrix.ptr<double>(0, 0);

  // create for loop to copy complete data
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = cameraInfo->K[i];
  }
}

// function to create lookup table for obtaining x,y,z values
void Tracker::createLookup(size_t width, size_t height)
{
  // get the values from the camera matrix of intrinsic parameters
  const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
  const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
  const float cx = cameraMatrixColor.at<double>(0, 2);
  const float cy = cameraMatrixColor.at<double>(1, 2);

  // float iterator
  float *it;

  // lookup table for y pixel locations
  lookupY = cv::Mat(1, height, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  // lookup table for x pixel locations
  lookupX = cv::Mat(1, width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}
