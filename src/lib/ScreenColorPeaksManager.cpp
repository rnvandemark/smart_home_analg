#include "smart_home_analg/ScreenColorPeaksManager.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace smart_home {

ScreenManager::ScreenManager(
	std::string screen_calibration_request_srv_topic,
	std::string cap_image_raw_sub_topic,
	std::string color_peak_left_pub_topic,
	std::string color_peak_right_pub_topic,
	std::string color_peaks_telem_pub_topic) :
		color_peak_left_msg(new smart_home_msgs::Color),
		color_peak_right_msg(new smart_home_msgs::Color),
		color_peaks_telem_msg(new smart_home_msgs::ColorPeaksTelem),
		last_received_frame(nullptr)
{
	screen_calibration_request_srv = nh.advertiseService(
		screen_calibration_request_srv_topic,
		&smart_home::ScreenManager::screen_calibration_request_callback,
		this
	);
	cap_image_raw_sub = nh.subscribe(
		cap_image_raw_sub_topic,
		1,
		&smart_home::ScreenManager::cap_image_raw_callback,
		this
	);
	color_peak_left_pub = nh.advertise<smart_home_msgs::Color>(
		color_peak_left_pub_topic,
		1
	);
	color_peak_right_pub = nh.advertise<smart_home_msgs::Color>(
		color_peak_right_pub_topic,
		1
	);
	color_peaks_telem_pub = nh.advertise<smart_home_msgs::ColorPeaksTelem>(
		color_peaks_telem_pub_topic,
		1
	);

	color_peak_left_msg->channels.resize(3, 0);
	color_peak_right_msg->channels.resize(3, 0);

	ROS_INFO("Initialized ScreenManager.");
}

bool ScreenManager::screen_calibration_request_callback(
	smart_home_msgs::RequestScreenCalibration::Request& req,
	smart_home_msgs::RequestScreenCalibration::Response& res)
{
	bool rc = false;
	cv_bridge::CvImagePtr cv_img_init;
	cv_bridge::CvImage cv_img_gray, cv_img_marked;
	std::vector<cv::Point2f> corners;

	const int max_corners = req.max_corners > 0 ? req.max_corners : 35;
	const float quality_level = req.quality_level > 0 ? req.quality_level : 0.01;
	const float min_dist = req.min_dist > 0 ? req.min_dist : 10;

	if (last_received_frame == nullptr)
	{
		goto END;
	}

	try
	{
		cv_img_init = cv_bridge::toCvCopy(last_received_frame, sensor_msgs::image_encodings::BGR8);
		res.intermediate_images.push_back(*(cv_img_init->toImageMsg()));
	}
	catch (cv_bridge::Exception& e)
	{
		goto END;
	}

	cv::cvtColor(cv_img_init->image, cv_img_gray.image, cv::COLOR_BGR2GRAY);
	cv_img_gray.header = cv_img_init->header;
	cv_img_gray.encoding = sensor_msgs::image_encodings::MONO8;
	res.intermediate_images.push_back(*(cv_img_gray.toImageMsg()));

	cv::goodFeaturesToTrack(cv_img_gray.image, corners, max_corners, quality_level, min_dist);

	cv_img_marked.image = cv_img_init->image.clone();
	cv_img_marked.header = cv_img_init->header;
	cv_img_marked.encoding = cv_img_init->encoding;
	for (std::vector<cv::Point2f>::iterator iter = corners.begin(); iter != corners.end(); ++iter)
	{
		int x = static_cast<int>(iter->x), y = static_cast<int>(iter->y);
		cv::circle(cv_img_marked.image, cv::Point(x,y), 5, CV_RGB(0,255,0));
		smart_home_msgs::Point pt;
		pt.x = x;
		pt.y = y;
		res.corner_positions.push_back(pt);
	}
	res.marked_image = *(cv_img_marked.toImageMsg());
	rc = true;
END:
	res.corners_found = rc;
	return true;
}

void ScreenManager::cap_image_raw_callback(const sensor_msgs::ImageConstPtr& msg)
{
	last_received_frame = msg;
}

}
