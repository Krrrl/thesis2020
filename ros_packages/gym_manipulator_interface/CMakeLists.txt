cmake_minimum_required(VERSION 2.8.3)
project(gym_manipulator_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(FILES
   ActionQueue.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES gym_manipulator_interface 
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
  DEPENDS 
)

#add_executable(gym_manipulator_interface ${PROJECT_SOURCE_DIR}/src/action_queue_server/action_server.py)
#target_link_libraries(gym_manipulator_interface ${catkin_LIBRARIES})

#add_executable(agent ${PROJECT_SOURCE_DIR}/src/random_agent/agent.py, ${PROJECT_SOURCE_DIR}/src/random_agent/agent_publisher.py)
#target_link_libraries(agent ${catkin_LIBRARIES})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/action_queue_server/action_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
#install(FILES
#	src/action_queue_server/action_server
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

