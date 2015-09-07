// processor.cpp: Class implementation to read rosbag files and obtain T-shirt point cloud
// Requirements: rosbag file as input
// Author: Nishanth Koganti
// Date: 2015/9/7

// tracker.cpp: Tracker class function declaration for tracking clothing articles
// using the cam shift algorithm and obtain point cloud using pcl functions
// Requirements: relies on the use of opencv and pcl
// Author: Nishanth Koganti
// Date: 2015/9/4

// TODO:
// 1) Improve point cloud processing using different filters

#include <tracker.h>

// class constructor
Tracker::Tracker()
{
  m_selectObject = false;
  cloudBag.open("cloud.bag", rosbag::bagmode::Write);
}

// class destructor
Tracker::~Tracker()
{
  cloudBag.close();
}

// class setter
void Tracker::setImages(cv::Mat &color, cv::Mat &depth, ros::Time time)
{
  // set the class variables from input images
  m_time = time;
  color.copyTo(m_color);
  depth.copyTo(m_depth);

  // perform cloud extraction on setting images
  cloudExtract();
}

// class getter
void Tracker::getOutput(cv::Mat &output, cv::Mat &backproj)
{
  // provide output images from class
  m_output.copyTo(output);
  m_backproj.copyTo(backproj);
}

// function to obtain cloth calibration values
void Tracker::clothCalibrate(cv::Mat &color, cv::Mat &depth)
{
	// opencv initialization
	char key = 0;
	cv::Mat disp, hsv, mask, hue;

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
	m_window = m_selection;
}

// function to display images
void Tracker::cloudExtract()
{
  // variable initialization
  cv::Mat roi;

  // function to extract image roi from color and depth functions
  createROI(roi);

  // function to create point cloud and obtain
  createCloud(roi);

  // write point cloud to rosbag file
  cloudBag.write("/kinect2/cloud", m_time, *(this->m_cloud));
}

// function to get image roi from color and depth images
void Tracker::createROI(cv::Mat &roi)
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
  window = m_window;

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
  m_window = window;

  window = cv::Rect(window.x - 15, window.y - 15, window.width + 15, window.height + 15);

  // draw rectangles around highlighted areas
  cv::rectangle(color, window.tl(), window.br(), cv::Scalar(255, 255, 255), 2, CV_AA);

  roi.release();

  pcMask = backproj(m_window);
  pcImg = depth(m_window);
  pcImg.copyTo(roi, pcMask);

  // copy color to output
  color.copyTo(m_output);
  backproj.copyTo(m_backproj);
}

// function to create point cloud from extracted ROI
void Tracker::createCloud(cv::Mat &roi)
{
  // initialize cloud
  m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // set cloud parameters
  m_cloud->header.frame_id = "cloth_frame";
  m_cloud->width = roi.cols;
  m_cloud->height = roi.rows;

  m_cloud->is_dense = false;
  m_cloud->points.resize(m_cloud->height * m_cloud->width);

  // variables
  const float badPoint = std::numeric_limits<float>::quiet_NaN();

  // parallel processing of pixel values
  #pragma omp parallel for
  for(int r = 0; r < roi.rows; ++r)
  {
    // create row of Points
    pcl::PointXYZ *itP = &m_cloud->points[r * roi.cols];

    // get pointer to row in depth image
    const cv::Vec3f *itD = roi.ptr< cv::Vec3f >(r);

    // convert all the depth values in the depth image to Points in point cloud
    for(size_t c = 0; c < (size_t)roi.cols; ++c, ++itP, ++itD)
    {
      float depthValue = (*itD).val[2];

      // Check for invalid measurements
      if(isnan(depthValue) || depthValue <= 0.001)
      {
        // set values to NaN for later processing
        itP->x = itP->y = itP->z = badPoint;
        continue;
      }

      // set the values for good points
      itP->z = (*itD).val[2];
      itP->x = (*itD).val[0];
      itP->y = (*itD).val[1];
    }
  }
}

// mouse click callback function for T-shirt color calibration
void Tracker::onMouse(int event, int x, int y, int flags, void* param)
{
  // this line needs to be added if we want to access the class private parameters
  // within a static function
  // URL: http://stackoverflow.com/questions/14062501/giving-callback-function-access-to-class-data-members-in-c
  Tracker* ptr = static_cast<Tracker*>(param);

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

// flag for write mode
bool writeMode = false;

// selecting default topic names when the options are not provided
std::string fileName = "kinect";

// parse command line arguments
for (int i = 0; i < argc; i++)
{
  std::string param = argv[i];

  if (param == "write")
    writeMode = true;
  else
    fileName = param;
}

// create rosbag file and create rosbag view instance
char bagName[200];

std::string topicColor = "/kinect2/color";
std::string topicDepth = "/kinect2/depth";

std::vector<std::string> topics;
topics.push_back(topicColor);
topics.push_back(topicDepth);

rosbag::Bag bag;
sprintf(bagName, "%s.bag", fileName.c_str());
bag.open(bagName, rosbag::bagmode::Read);
rosbag::View view(bag, rosbag::TopicQuery(topics));

// rosbag parse variables
int frame = 0;
ros::Time time;
rosbag::View::iterator iter = view.begin();

// opencv variables
cv::Mat color, depth, output, backproj, roi;
pcl::PointCloud< pcl::PointXYZ > cloud;

// named windows
cv::namedWindow("Output",1);
cv::namedWindow("Backproj",1);

// create instance of tracker class
Tracker tracker;

// get first pair of images for calibration
for (int i = 0; i < topics.size(); i++)
{
  rosbag::MessageInstance const m = *iter;

  sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
  if (image->encoding == "bgr8")
    readImage(image, color);
  else if (image->encoding == "32FC3")
    readImage(image, depth);

  ++iter;
}

// perform calibration
tracker.clothCalibrate(color, depth);

// create video writer
cv::VideoWriter writer;
int width = depth.cols;
int height = depth.rows;
if (writeMode)
{
  writer.open ("output.avi", CV_FOURCC('D','I','V','X'), FPS, cv::Size (width/2,height/2), true );
}

cv::Rect rect;
rect.x = width/4;
rect.y = height/2 - 50;
rect.width = width/2;
rect.height = height/2;

// create pcl cloud viewer
pcl::visualization::CloudViewer viewer("Cloth Tracker");

// main rosbag parse loop
while(iter != view.end())
{
  // get time for msgs and pair of color and depth images
  time = (*iter).getTime();

  for (int i = 0; i < topics.size(); i++)
  {
    rosbag::MessageInstance const m = *iter;

    sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
    if (image->encoding == "bgr8")
      readImage(image, color);
    else if (image->encoding == "32FC3")
      readImage(image, depth);

    ++iter;
  }

  // process images and get output
  tracker.setImages(color, depth, time);
  tracker.getOutput(output, backproj);
  roi = output(rect);

  // show constructed point cloud
  viewer.showCloud(tracker.m_cloud);

  // write output image to videoWriter
  if (writeMode)
  {
    writer.write(roi);
  }

  // display output images
  cv::imshow("Output", roi);
  cv::imshow("Backproj", backproj);
  cv::waitKey(30);

  std::cout << "Frame: " << frame << ", Time: " << time << std::endl;
  frame++;
}

// close instances
bag.close();
writer.release();
