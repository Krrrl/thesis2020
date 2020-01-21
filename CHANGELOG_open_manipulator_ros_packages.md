# CHANGELOG

This is the CHANGELOG for my additions and changes to the open_manipulator packages supplied by ROBOTIS.

Changes made:

### open_manipulator_msgs :
	-Added file srv/ActionQueue.srv
	-Modified CMakeLists.txt
		-add_service_files( .... ActionQueue.srv)


### open_manipulator_teleop :
	-Added file src/open_manipulator_teleop_gym.cpp
	-Added file include/open_manipulator_teleop/open_manipulator_teleop_gym.h
	-Added file launch/open_manipulator_teleop_gym.launch

	-Modified CMakeLists.txt
		-add_executable(open_manipulator_teleop_gym src/open_manipulator_teleop_gym.cpp)
		-add_dependencies(open_manipulator_teleop_gym ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
		-target_link_libraries(open_manipulator_teleop_gym ${catkin_LIBRARIES})
	
	-Modified CMakeLists.txt
		-install(TARGETS .. .. open_manipulator_teleop_gym ..)
