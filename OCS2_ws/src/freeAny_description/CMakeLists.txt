cmake_minimum_required(VERSION 2.8.3)

project(freeAny_description)

find_package(catkin REQUIRED)

catkin_package(
	CATKIN_DEPENDS
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
