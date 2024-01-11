cmake_minimum_required(VERSION 3.10.0)

project(ranger_mini)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
