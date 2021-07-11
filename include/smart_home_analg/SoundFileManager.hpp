#ifndef __SMART_HOME_ANALG_SOUNDFILEMANAGER_HPP
#define __SMART_HOME_ANALG_SOUNDFILEMANAGER_HPP

#include "smart_home_analg/SoundHelpers.hpp"

#include <string>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <smart_home_msgs/StringArr.h>
#include <smart_home_msgs/PlaybackCommand.h>
#include <smart_home_msgs/Float32Arr.h>

namespace smart_home {

class SfManager
{
protected:
	ros::NodeHandle nh;
	ros::Subscriber sound_file_path_sub;
	ros::Subscriber sound_file_path_list_sub;
	ros::Subscriber fft_window_sub;
	ros::Subscriber playback_command_sub;
	ros::Publisher playback_frequencies_pub;
	ros::Timer calc_frequencies_tmr;

	smart_home_msgs::Float32ArrPtr playback_frequencies_msg;

	const std::string root_dir;
	const int max_num_freqs;

	std::queue<std::string> queued_sound_file_paths;
	float current_fft_window;
	std::string current_sf_name;

	struct SfPlaybackData; // Forward declare
	bool ready; // Whether or not the value of sfpd is valid
	SfPlaybackData* sfpd;

	void queue_sound_file(const std::string& sf_path);
	void pop_next_playback();
	std::vector<float> get_max_freqs(int chunk);

public:
	SfManager(
		const std::string root_dir,
		const int max_num_freqs,
		const float init_fft_window,
		const std::string sound_file_path_sub_topic,
		const std::string sound_file_path_list_sub_topic,
		const std::string fft_window_sub_topic,
		const std::string playback_command_sub_topic,
		const std::string playback_frequencies_pub_topic
	);
	~SfManager();

	void sound_file_path_callback(const std_msgs::String& msg);
	void sound_file_path_list_callback(const smart_home_msgs::StringArr& msg);
	void fft_window_callback(const std_msgs::Float32& msg);
	void playback_command_callback(const smart_home_msgs::PlaybackCommand& msg);
	void calc_frequencies_callback(const ros::TimerEvent& evt);
};

}

#endif // __SMART_HOME_ANALG_SOUNDFILEMANAGER_HPP
