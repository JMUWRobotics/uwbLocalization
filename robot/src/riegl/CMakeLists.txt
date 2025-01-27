cmake_minimum_required(VERSION 2.8.3)
project(riegl)


set(RIEGL_DIR ${CMAKE_SOURCE_DIR}/rivlib/)
message("CMAKE_SOURCE_DIR" ${CMAKE_SOURCE_DIR})

set(RiVLib_USE_STATIC_RUNTIME ON)
find_package(RiVLib REQUIRED COMPONENTS scanlib ctrllib HINTS "${RIEGL_DIR}/cmake/")
include_directories(${RiVLib_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
  geometry_msgs
  tf
)

add_message_files(
  FILES
  RieglStatus.msg
  RieglTime.msg
)

add_service_files(
  FILES
  angle.srv
  scanparams.srv
  Command.srv
  selection.srv
  progress.srv
  inclination.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime roscpp
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories(
  lib
)

add_executable(rxplogger src/rxplogger.cc)
set_target_properties(rxplogger PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(rxplogger
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(rxplogger riegl_generate_messages_cpp)

add_executable(rxploggersingle src/rxploggersingle.cc)
set_target_properties(rxploggersingle PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(rxploggersingle
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(rxploggersingle riegl_generate_messages_cpp)

add_executable(raw2scan src/raw2scan.cc)
set_target_properties(raw2scan PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(raw2scan
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(raw2scan riegl_generate_messages_cpp)

add_executable(ctr src/rieglctrl.cc)
set_target_properties(ctr PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(ctr
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(ctr riegl_generate_messages_cpp)

add_executable(status src/status.cc)
set_target_properties(status PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(status
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(status riegl_generate_messages_cpp)

add_executable(receiver src/receiver.cc)
set_target_properties(receiver PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(receiver riegl_generate_messages_cpp)

add_executable(receiver2D src/receiver2D.cc)
set_target_properties(receiver2D PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver2D
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(receiver2D riegl_generate_messages_cpp)

add_executable(receiver3D src/receiver3D.cc)
set_target_properties(receiver3D PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver3D
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(receiver3D riegl_generate_messages_cpp)

add_executable(receiver4D src/receiver4D.cc)
set_target_properties(receiver4D PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiver4D
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(receiver4D riegl_generate_messages_cpp)

add_executable(receiverESA src/receiverESA.cc)
set_target_properties(receiverESA PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(receiverESA
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(receiverESA riegl_generate_messages_cpp)

add_executable(tslogger src/tslogger.cc)
set_target_properties(tslogger PROPERTIES LINK_FLAGS "-pthread")
target_link_libraries(tslogger
  ${catkin_LIBRARIES}
  ${RiVLib_CTRLLIB_LIBRARY}
  ${RiVLib_SCANLIB_LIBRARY}
)
add_dependencies(tslogger riegl_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}
	DESTINATION ${CATKIN_DEVEL_PREFIX}/include
	PATTERN ".svn" EXCLUDE)

install(DIRECTORY lib/
	#DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	DESTINATION ${CATKIN_DEVEL_PREFIX}/lib
	PATTERN ".svn" EXCLUDE)
