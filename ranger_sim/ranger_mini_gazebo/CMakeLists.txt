cmake_minimum_required(VERSION 3.10.0)

project(ranger_mini_gazebo)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir description launch worlds xacro)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
