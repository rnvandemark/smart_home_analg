#ifndef __SMART_HOME_ANALG_SCREENCOLORPEAKSMANAGER_HPP
#define __SMART_HOME_ANALG_SCREENCOLORPEAKSMANAGER_HPP

#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <smart_home_msgs/Color.h>
#include <smart_home_msgs/ColorPeaksTelem.h>
#include <smart_home_msgs/RequestScreenCalibration.h>

namespace smart_home {

class ScreenManager
{
protected:
	ros::NodeHandle nh;
	ros::ServiceServer screen_calibration_request_srv;
	ros::Subscriber cap_image_raw_sub;
	ros::Publisher color_peak_left_pub;
	ros::Publisher color_peak_right_pub;
	ros::Publisher color_peaks_telem_pub;

	smart_home_msgs::ColorPtr color_peak_left_msg;
	smart_home_msgs::ColorPtr color_peak_right_msg;
	smart_home_msgs::ColorPeaksTelemPtr color_peaks_telem_msg;

	sensor_msgs::ImageConstPtr last_received_frame;

public:
	ScreenManager(
		std::string screen_calibration_request_srv_topic,
		std::string cap_image_raw_sub_topic,
		std::string color_peak_left_pub_topic,
		std::string color_peak_right_pub_topic,
		std::string color_peaks_telem_pub_topic);

	bool screen_calibration_request_callback(
		smart_home_msgs::RequestScreenCalibration::Request& req,
		smart_home_msgs::RequestScreenCalibration::Response& res);

	void cap_image_raw_callback(const sensor_msgs::ImageConstPtr& msg);
};

}

#endif // __SMART_HOME_ANALG_SCREENCOLORPEAKSMANAGER_HPP
