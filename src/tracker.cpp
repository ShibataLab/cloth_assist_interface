// tracker.cpp: Tracker class function declaration for tracking clothing articles
// using the cam shift algorithm and obtain point cloud using pcl functions
// Requirements: relies on the use of iai kinect2 bridge, opencv and pcl
// Author: Nishanth Koganti
// Date: 2015/10/27

// TODO:
// 1) Improve point cloud processing using different filters

#include <tracker.h>

// class constructor
Tracker::Tracker(const std::string &topicColor, const std::string &topicDepth, const std::string &topicType, const std::string &calibFile)
    : _topicColor(topicColor), _topicDepth(topicDepth), _topicType(topicType), _calibFile(calibFile), _updateImage(false), _running(false), _queueSize(5), _nh("~"), _spinner(0), _it(_nh)
{
  // create matrices for intrinsic parameters
  _cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
  _cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);

  // set the point cloud processor properties
  _sor.setMeanK(100);
  _sor.setStddevMulThresh(0.7);
  _vog.setLeafSize(0.015f, 0.015f, 0.015f);

  // get the camera info ros topics for color and depth
  _topicCameraInfoColor = _topicColor.substr(0, _topicColor.rfind('/')) + "/camera_info";
  _topicCameraInfoDepth = _topicDepth.substr(0, _topicDepth.rfind('/')) + "/camera_info";

  // setting the transform parameter
  _transform = Eigen::Matrix4f::Identity();

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
  _running = true;

  // TransportHints stores the transport settings for the image topic subscriber
  // here we are giving the setting of raw or compressed using the useCompressed variable
  image_transport::TransportHints hints("raw");

  // SubscriberFilters are used to subscribe to the kinect image topics
  _subImageColor = new image_transport::SubscriberFilter(_it, _topicColor, _queueSize, hints);
  _subImageDepth = new image_transport::SubscriberFilter(_it, _topicDepth, _queueSize, hints);

  // message filters are used to subscribe to the camera info topics
  _subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(_nh, _topicCameraInfoColor, _queueSize);
  _subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(_nh, _topicCameraInfoDepth, _queueSize);

  // create ros publisher
  _pubTrackImage = _it.advertise("/cloth/image", 10);
  _pubPointCloud = _nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/cloth/points", 10);
  _pubESFDescriptor = _nh.advertise<pcl::PointCloud<pcl::ESFSignature640> > ("/cloth/descriptor", 10);

  // creating a exact synchronizer for 4 ros topics with queueSize
  // the Tracker class callback function is set as the callback function
  _syncExact = new message_filters::Synchronizer<_ExactSyncPolicy>(_ExactSyncPolicy(_queueSize), *_subImageColor, *_subImageDepth, *_subCameraInfoColor, *_subCameraInfoDepth);
  _syncExact->registerCallback(boost::bind(&Tracker::callback, this, _1, _2, _3, _4));

  // set the width and height parameters for cloth tracking functions
  if(_topicType == "hd")
  {
    _height = 1080;
    _width = 1920;
  }
  else if(_topicType == "qhd")
  {
    _height = 540;
    _width = 960;
  }
  else if(_topicType == "sd")
  {
    _height = 424;
    _width = 512;
  }

  // from here we start the 4 threads
  _spinner.start();

  // create a chrono instance for 1 millisecond
  std::chrono::milliseconds duration(1);

  // if the updateImage flags are not set then we will check for ros shutdown
  while(!_updateImage)
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
  _spinner.stop();

  // clean up all variables
  delete _syncExact;

  delete _subImageColor;
  delete _subImageDepth;
  delete _subCameraInfoColor;
  delete _subCameraInfoDepth;

  _running = false;
}

// message filter callback function
void Tracker::callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
              const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
{
  // initialization
  cv::Mat color, depth;

  // get the camera info and images
  readCameraInfo(cameraInfoColor, _cameraMatrixColor);
  readCameraInfo(cameraInfoDepth, _cameraMatrixDepth);
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
  _lock.lock();
  _color = color;
  _depth = depth;
  _updateImage = true;
  _lock.unlock();
}

// function to obtain cloth calibration values
// cloth calibrate function
void Tracker::clothCalibrate()
{
	// opencv initialization
	char key = 0;
	cv::Mat color, depth, disp, hsv, mask, hue;

	// GUI initialization
	cv::namedWindow("CamShift", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("CamShift", onMouse, (void *)this);

	// get color and depth images
  for(; _running && ros::ok();)
  {
    if(_updateImage)
    {
      // _lock the class variables
      _lock.lock();
      color = _color;
      depth = _depth;
      _updateImage = false;
      _lock.unlock();
      break;
    }
  }

  color.copyTo(disp);

  // inifinite for loop
  for(; _running && ros::ok();)
  {
		// show the selection
		if (_selectObject && _selection.width > 0 && _selection.height > 0)
		{
			color.copyTo(disp);
			cv::Mat roi(disp, _selection);
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
	cv::Mat roi(hue, _selection), maskroi(mask, _selection);

	// compute the histogram and normaize the histogram
	cv::calcHist(&roi, 1, 0, maskroi, _hist, 1, &hsize, &phranges);
	cv::normalize(_hist, _hist, 0, 255, cv::NORM_MINMAX);

	// track window initialization
	_window = _selection;
}

// function to display images
void Tracker::clothTracker()
{
  // variable initialization
  cv::Rect window;
  cv::Mat color, depth, roi, backproj;

  // create chrono instances to measure performance of the program
  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;

  // load the calibration parameters
  if (_calibFile != "default")
  {
    char c;
    ifstream calibDat(_calibFile, ifstream::in);
    for (int i = 0; i < 4; i++)
      calibDat >> _transform(i,0) >> c >> _transform(i,1) >> c >> _transform(i,2) >> c >> _transform(i,3);
    std::cout << _transform << std::endl;
  }

  // more variables for printing text and other functions
  double fps = 0;
  size_t frameCount = 0;

  // create named windows for displaying color and backprojection images
  // cv::namedWindow("Color");
  // cv::namedWindow("Backproj");

  // pcl initialization
  // pcl::visualization::CloudViewer viewer("Cloth Point Cloud");

  // obtain starting time point
  start = std::chrono::high_resolution_clock::now();

  // inifinite for loop
  for(; _running && ros::ok();)
  {
    // check for image update
    if(_updateImage)
    {
      // _lock the class variables
      _lock.lock();
      color = _color;
      depth = _depth;
      _updateImage = false;
      _lock.unlock();

      ++frameCount;

      // get the time elapsed so far
      now = std::chrono::high_resolution_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;

      if(elapsed >= 1.0)
      {
        fps = frameCount / elapsed;
        std::cout << fps << std::endl;
        start = now;
        frameCount = 0;
      }

      // function to extract image roi from color and depth functions
      createROI(color, depth, backproj, roi);

      // create lookup tables for x and y mappings
      createLookup(_width, _height);

      // function to create point cloud and obtain
      createCloud(roi);

      // function to process the point cloud
      processCloud();

      // create color image msg
      sensor_msgs::ImagePtr colorMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();

      // publish the point cloud message
      _pubTrackImage.publish(colorMsg);
      _pubESFDescriptor.publish(_cloudESF);
      _pubPointCloud.publish(_cloudCentered);

      // show image
      // cv::imshow("Color", color);
      // cv::imshow("Backproj", backproj);

      // display point cloud
      // if (_cloud->size() > 0)
      //   viewer.showCloud(_cloudCentered);
    }

    // wait for key press
    int key = cv::waitKey(1);

    // check for different key presses
    switch(key & 0xFF)
    {
      case 27:
      case 'q':
        _running = false;
        break;
      case ' ':
        break;
    }
  }

  // destroy all windows and shutdown
  std::cout << std::endl;
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
  cv::Rect boundRect = cv::Rect(0, 0, _width, _height);

  // initialize the calibration values
  hist = _hist;
  window = _window;
  color = _color;

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
    window = boundRect;
  _window = window;

  window = cv::Rect(window.x - 15, window.y - 15, window.width + 15, window.height + 15) & boundRect;

  // draw rectangles around highlighted areas
  cv::rectangle(color, window.tl(), window.br(), cv::Scalar(255, 255, 255), 2, CV_AA);

  roi.release();

  pcImg = depth(window);
  pcMask = backproj(window);
  pcImg.copyTo(roi, pcMask);
}

// function to create point cloud from extracted ROI
void Tracker::createCloud(cv::Mat &roi)
{
  // initialize cloud
  _cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // set cloud parameters
  _cloud->header.frame_id = "kinect2_link";
  _cloud->width = roi.cols;
  _cloud->height = roi.rows;

  _cloud->is_dense = false;
  _cloud->points.resize(_cloud->height * _cloud->width);

  // variables
  cv::Rect window = _window;
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  // parallel processing of pixel values
  #pragma omp parallel for
  for(int r = 0; r < roi.rows; ++r)
  {
    // create row of Points
    pcl::PointXYZ *itP = &_cloud->points[r * roi.cols];

    // get pointer to row in depth image
    const uint16_t *itD = roi.ptr<uint16_t>(r);

    // get the x and y values
    const float y = _lookupY.at<float>(0, window.y+r);
    const float *itX = _lookupX.ptr<float>();
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

// function to process point cloud
void Tracker::processCloud()
{
  // create new instance of the point cloud
  _cloudVOG = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  _cloudSOR = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  _cloudCentered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  _cloudTransform = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  _cloudESF = pcl::PointCloud<pcl::ESFSignature640>::Ptr(new pcl::PointCloud<pcl::ESFSignature640>());

  // process the point cloud
  _vog.setInputCloud(_cloud);
  _vog.filter(*_cloudVOG);

  _sor.setInputCloud(_cloudVOG);
  _sor.filter(*_cloudSOR);

  // Center point cloud
  pcl::transformPointCloud(*_cloudSOR, *_cloudTransform, _transform);
  pcl::compute3DCentroid(*_cloudTransform, _centroid);
  pcl::demeanPointCloud(*_cloudTransform, _centroid, *_cloudCentered);

  // Compute ESF descriptor
  _esf.setInputCloud(_cloudCentered);
  _esf.compute(*_cloudESF);

  // recompute centered without transformation
  pcl::compute3DCentroid(*_cloudSOR, _centroid);
  pcl::demeanPointCloud(*_cloudSOR, _centroid, *_cloudCentered);
}

// mouse click callback function for T-shirt color calibration
void Tracker::onMouse(int event, int x, int y, int flags, void* param)
{
  // this line needs to be added if we want to access the class private parameters
  // within a static function
  // URL: http://stackoverflow.com/questions/14062501/giving-callback-function-access-to-class-data-members-in-c
  Tracker* ptr = (Tracker*) param;

	if (ptr->_selectObject)
	{
		ptr->_selection.x = std::min(x, ptr->_origin.x);
		ptr->_selection.y = std::min(y, ptr->_origin.y);
		ptr->_selection.width = std::abs(x - ptr->_origin.x);
		ptr->_selection.height = std::abs(y - ptr->_origin.y);
		ptr->_selection &= cv::Rect(0, 0, ptr->_width, ptr->_height);
	}

	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		ptr->_origin = cv::Point(x, y);
		ptr->_selection = cv::Rect(x, y, 0, 0);
		ptr->_selectObject = true;
		break;
	case cv::EVENT_LBUTTONUP:
		ptr->_selectObject = false;
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
  const float fx = 1.0f / _cameraMatrixColor.at<double>(0, 0);
  const float fy = 1.0f / _cameraMatrixColor.at<double>(1, 1);
  const float cx = _cameraMatrixColor.at<double>(0, 2);
  const float cy = _cameraMatrixColor.at<double>(1, 2);

  // float iterator
  float *it;

  // lookup table for y pixel locations
  _lookupY = cv::Mat(1, height, CV_32F);
  it = _lookupY.ptr<float>();
  for(size_t r = 0; r < height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  // lookup table for x pixel locations
  _lookupX = cv::Mat(1, width, CV_32F);
  it = _lookupX.ptr<float>();
  for(size_t c = 0; c < width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}
