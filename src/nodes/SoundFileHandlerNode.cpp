#include <iostream>
#include <ros/ros.h>
#include "smart_home_analg/SoundFileManager.hpp"

static const std::string NODE_NAME = "smart_home_sound_file_handler";

int main(int argc, char** argv) 
{ 
	ros::init(argc, argv, NODE_NAME);
	if (argc != 9)
	{
		std::cerr << "Expected exactly seven args (frequency count, initial FFT window, sound file path sub topic,"
						" sound file path list sub topic, FFT window sub topic, playback commands sub topic,"
						" max frequencies pub topic, playback updates topic), got " << (argc-1) << std::endl;
		return -1;
	}
	smart_home::SfManager manager(std::stoi(argv[1]), std::stof(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7], argv[8]);
	ros::spin();
	return 0;
}
