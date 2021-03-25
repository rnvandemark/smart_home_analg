cmake_minimum_required(VERSION 3.0.2)
project(smart_home_analg)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	sensor_msgs
	cv_bridge
	image_transport
	smart_home_msgs
)
find_package(OpenCV)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport smart_home_msgs std_msgs
	DEPENDS OpenCV
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)
