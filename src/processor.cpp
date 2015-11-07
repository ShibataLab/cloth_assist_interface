// processor.cpp: Class implementation to read rosbag files and obtain T-shirt point cloud
// Requirements: rosbag file as input
// Author: Nishanth Koganti
// Date: 2015/9/7

// TODO:
// 1) Improve point cloud processing using different filters

#include <processor.h>

// class constructor
Processor::Processor(std::string fileName, std::string topicColor, std::string topicDepth, std::string topicCameraInfo, std::string topicType, bool videoMode, bool cloudMode)
  : m_videoMode(videoMode), m_cloudMode(cloudMode)
{
  // initialize select object flag
  m_trackMode = true;
  m_selectObject = false;

  // create matrices for intrinsic parameters
  m_cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

  // create char file names
  char bagName[200], cloudBagName[200], videoName[200], tracksName[200];

  // open the bag file
  sprintf(bagName, "%s.bag", fileName.c_str());
  m_bag.open(bagName, rosbag::bagmode::Read);

  // create vector of topics for querying
  std::vector<std::string> topics;
  topics.push_back(topicColor);
  topics.push_back(topicDepth);
  topics.push_back(topicCameraInfo);

  // set the filter length
  m_filterLength = 4;

  // create view instance for rosbag parsing
  m_view = new rosbag::View(m_bag, rosbag::TopicQuery(topics));

  // set the width and height parameters for cloth tracking functions
  if(topicType == "hd")
  {
    m_height = 1080;
    m_width = 1920;
  }
  else if(topicType == "qhd")
  {
    m_height = 540;
    m_width = 960;
  }
  else if(topicType == "sd")
  {
    m_height = 424;
    m_width = 512;
  }

  // trackMode
  if (m_trackMode)
  {
    sprintf(tracksName, "%s", fileName.c_str());
    m_tracks.open(tracksName);
    m_tracks << "Frame,Time" << std::endl;
  }

  // cloudMode
  if (m_cloudMode)
  {
    sprintf(cloudBagName, "%sCloud.bag", fileName.c_str());
    m_cloudBag.open(cloudBagName, rosbag::bagmode::Write);
  }

  // videoMode
  if (m_videoMode)
  {
    sprintf(videoName, "%s.avi", fileName.c_str());
    m_writer.open (videoName, CV_FOURCC('D','I','V','X'), FPS, cv::Size (m_width,m_height), true);
  }
}

// class destructor
Processor::~Processor()
{
}

// write run function
void Processor::run()
{
  // opencv variables
  cv::Mat roi;
  cv::namedWindow("Output", 1);
  cv::namedWindow("Backproj", 1);

  // start parsing rosbag
  std::string type;
  std::string topicCloud = "/cloth/cloud";
  rosbag::View::iterator iter = m_view->begin();

  double currentTime, timeTrack, startTime;

  // create pcl cloud viewer
  pcl::visualization::CloudViewer viewer("Cloth Tracker");

  // ros time instance
  m_time = (*iter).getTime();
  startTime = m_time.toSec();

  // read first set of images for calibration
  for (int i = 0; i < 3; i++)
  {
    rosbag::MessageInstance const m = *iter;

    type = m.getDataType();
    if (type == "sensor_msgs/Image")
    {
      sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
      if (image->encoding == "bgr8")
        readImage(image, m_color);
      else if (image->encoding == "16UC1")
        readImage(image, m_depth);
    }
    else
      m_cameraInfo = m.instantiate<sensor_msgs::CameraInfo>();

    ++iter;
  }

  // peform cloth calibration
  clothCalibrate();

  // main loop
  m_frame = 1;
  while(iter != m_view->end())
  {
    m_time = (*iter).getTime();
    currentTime = m_time.toSec();
    timeTrack = currentTime - startTime;

    for (int i = 0; i < 3; i++)
    {
      rosbag::MessageInstance const m = *iter;

      type = m.getDataType();
      if (type == "sensor_msgs/Image")
      {
        sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
        if (image->encoding == "bgr8")
          readImage(image, m_color);
        else if (image->encoding == "16UC1")
          readImage(image, m_depth);
      }
      else
        m_cameraInfo = m.instantiate<sensor_msgs::CameraInfo>();

      ++iter;
    }

    cloudExtract();

    if (m_trackMode)
      m_tracks << m_frame << "," << timeTrack << std::endl;

    if (m_videoMode)
      m_writer.write(m_output);

    if (m_cloudMode)
      m_cloudBag.write(topicCloud, m_time, *m_cloud);

    viewer.showCloud(m_cloud);
    cv::imshow("Output", m_output);
    cv::imshow("Backproj", m_backproj);
    cv::waitKey(20);

    m_frame++;
  }

  // clean exit
  m_bag.close();
  m_writer.release();

  if (m_cloudMode)
    m_cloudBag.close();

}

// function to obtain cloth calibration values
void Processor::clothCalibrate()
{
	// opencv initialization
	char key = 0;
	cv::Mat disp, hsv, mask, hue, color;

  // get color image
  m_color.copyTo(color);

	// gui initialization
	cv::namedWindow("CamShift", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("CamShift", onMouse, (void *)this);

	// create a copy of color image
  color.copyTo(disp);

  // inifinite for loop
  while(ros::ok())
  {
		// show the selection
		if (m_selectObject && m_selection.width > 0 && m_selection.height > 0)
		{
			color.copyTo(disp);
			cv::Mat roi(disp, m_selection);
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
	cv::Mat roi(hue, m_selection), maskroi(mask, m_selection);

	// compute the histogram and normaize the histogram
	cv::calcHist(&roi, 1, 0, maskroi, m_hist, 1, &hsize, &phranges);
	cv::normalize(m_hist, m_hist, 0, 255, cv::NORM_MINMAX);

	// track window initialization
	m_rawWindow = m_selection;
  m_windows.push_back(m_rawWindow);
}

// function to display images
void Processor::cloudExtract()
{
  // variable initialization
  cv::Mat roi;

  // function to extract image roi from color and depth functions
  createROI(roi);

  // function to create point cloud and obtain
  createCloud(roi);
}

// function to get image roi from color and depth images
void Processor::createROI(cv::Mat &roi)
{
  // opencv initialization
  cv::Mat color, depth, backproj;

  // copy images to function images
  m_color.copyTo(color);
  m_depth.copyTo(depth);

  // cam shift initialization
  cv::Rect window;
  int ch[] = { 0, 0 };
  cv::RotatedRect box;
  float hranges[] = { 0, 180 };
  const float* phranges = hranges;
  cv::Mat hsv, hue, mask, hist, pcImg, pcMask;

  // initialize the calibration values
  hist = m_hist;
  window = m_rawWindow;

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
    window = cv::Rect(0, 0, m_width, m_height);
  m_rawWindow = window;

  int rectX = 0, rectY = 0, rectW = 0, rectH = 0;
  if (m_frame > m_filterLength-1)
  {
    for (int i = 0; i < m_filterLength; i++)
    {
      rectX += m_windows[i].x/(m_filterLength+1);
      rectY += m_windows[i].y/(m_filterLength+1);
      rectW += m_windows[i].width/(m_filterLength+1);
      rectH += m_windows[i].height/(m_filterLength+1);
    }

    rectX += window.x/(m_filterLength+1);
    rectY += window.y/(m_filterLength+1);
    rectW += window.width/(m_filterLength+1);
    rectH += window.height/(m_filterLength+1);

    m_window = cv::Rect(rectX, rectY, rectW+10, rectH+10);
    m_windows.erase(m_windows.begin(), m_windows.begin()+1);
  }
  else
    m_window = window;

  m_windows.push_back(m_window);

  // draw rectangles around highlighted areas
  cv::rectangle(color, window.tl(), window.br(), cv::Scalar(0, 0, 255), 2, CV_AA);
  cv::rectangle(color, m_window.tl(), m_window.br(), cv::Scalar(0, 255, 0), 2, CV_AA);

  roi.release();

  pcMask = backproj(m_window);
  pcImg = depth(m_window);
  pcImg.copyTo(roi, pcMask);

  // copy color to output
  color.copyTo(m_output);
  backproj.copyTo(m_backproj);
}

// function to create point cloud from extracted ROI
void Processor::createCloud(cv::Mat &roi)
{
  // initialize cloud
  m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // set cloud parameters
  m_cloud->header.frame_id = "cloth_frame";
  m_cloud->width = roi.cols;
  m_cloud->height = roi.rows;

  m_cloud->is_dense = false;
  m_cloud->points.resize(m_cloud->height * m_cloud->width);

  // create lookup tables
  readCameraInfo();

  createLookup();

  // variables
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  // parallel processing of pixel values
  #pragma omp parallel for
  for(int r = 0; r < roi.rows; ++r)
  {
    // create row of Points
    pcl::PointXYZ *itP = &m_cloud->points[r * roi.cols];

    // get pointer to row in depth image
    const uint16_t *itD = roi.ptr<uint16_t>(r);

    // get the x and y values
    const float y = m_lookupY.at<float>(0, m_window.y+r);
    const float *itX = m_lookupX.ptr<float>();
    itX = itX + m_window.x;

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
void Processor::onMouse(int event, int x, int y, int flags, void* param)
{
  // this line needs to be added if we want to access the class private parameters
  // within a static function
  // URL: http://stackoverflow.com/questions/14062501/giving-callback-function-access-to-class-data-members-in-c
  Processor* ptr = static_cast<Processor*>(param);

	if (ptr->m_selectObject)
	{
		ptr->m_selection.x = std::min(x, ptr->m_origin.x);
		ptr->m_selection.y = std::min(y, ptr->m_origin.y);
		ptr->m_selection.width = std::abs(x - ptr->m_origin.x);
		ptr->m_selection.height = std::abs(y - ptr->m_origin.y);
		ptr->m_selection &= cv::Rect(0, 0, ptr->m_width, ptr->m_height);
	}

	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		ptr->m_origin = cv::Point(x, y);
		ptr->m_selection = cv::Rect(x, y, 0, 0);
		ptr->m_selectObject = true;
		break;
	case cv::EVENT_LBUTTONUP:
		ptr->m_selectObject = false;
		break;
	}
}

void Processor::readImage(sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
  // obtain image data and encoding from sensor msg
  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);

  // copy data to the Mat image
  pCvImage->image.copyTo(image);
}

// function to obtain camera info from message filter msgs
void Processor::readCameraInfo()
{
  // get pointer for first element in cameraMatrix
  double *itC = m_cameraMatrix.ptr<double>(0, 0);

  // create for loop to copy complete data
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    *itC = m_cameraInfo->K[i];
  }
}

// function to create lookup table for obtaining x,y,z values
void Processor::createLookup()
{
  // get the values from the camera matrix of intrinsic parameters
  const float fx = 1.0f / m_cameraMatrix.at<double>(0, 0);
  const float fy = 1.0f / m_cameraMatrix.at<double>(1, 1);
  const float cx = m_cameraMatrix.at<double>(0, 2);
  const float cy = m_cameraMatrix.at<double>(1, 2);

  // float iterator
  float *it;

  // lookup table for y pixel locations
  m_lookupY = cv::Mat(1, m_height, CV_32F);
  it = m_lookupY.ptr<float>();
  for(size_t r = 0; r < m_height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  // lookup table for x pixel locations
  m_lookupX = cv::Mat(1, m_width, CV_32F);
  it = m_lookupX.ptr<float>();
  for(size_t c = 0; c < m_width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}
