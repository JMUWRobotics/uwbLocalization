cmake_minimum_required(VERSION 3.22)
project(DecaWave)

#set(TMP_EXECUTABLE uwb_tmp)
set(TAG_EXECUTABLE uwb_tag)
set(ANCHOR_EXECUTABLE uwb_anchor)
set(ID_EXECUTABLE uwb_id)
set(MONITOR_EXECUTABLE uwb_monitor)

set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

#file(GLOB TMP_SRC_FILES *.cpp decadriver/*.c wrapper/*.cpp ros_lib/*.cpp src/common/*.cpp src/tmp/*.cpp)
file(GLOB TAG_SRC_FILES *.cpp decadriver/*.c wrapper/*.cpp ros_lib/*.cpp src/common/*.cpp src/tag/*.cpp)
file(GLOB ANCHOR_SRC_FILES *.cpp decadriver/*.c wrapper/*.cpp ros_lib/*.cpp src/common/*.cpp src/anchor/*.cpp)
file(GLOB ID_SRC_FILES *.cpp decadriver/*.c wrapper/*.cpp ros_lib/*.cpp src/common/*.cpp src/id/*.cpp)
file(GLOB MONITOR_SRC_FILES *.cpp decadriver/*.c wrapper/*.cpp ros_lib/*.cpp src/common/*.cpp src/monitor/*.cpp)

#file(GLOB TMP_INCLUDE_FILES *.h decadriver/*.h wrapper/*.h ros_lib/*.h include/common/*.h include/tmp/*.h)
file(GLOB TAG_INCLUDE_FILES *.h decadriver/*.h wrapper/*.h ros_lib/*.h include/common/*.h include/tag/*.h)
file(GLOB ANCHOR_INCLUDE_FILES *.h decadriver/*.h wrapper/*.h ros_lib/*.h include/common/*.h include/anchor/*.h)
file(GLOB ID_INCLUDE_FILES *.h decadriver/*.h wrapper/*.h ros_lib/*.h include/common/*.h include/id/*.h)
file(GLOB MONITOR_INCLUDE_FILES *.h decadriver/*.h wrapper/*.h ros_lib/*.h include/common/*.h include/monitor/*.h)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
add_subdirectory(rodos)
# add_subdirectory(ros)
# add_rodos_executable(${EXECUTABLE_NAME} ${SRC_FILES} ${INCLUDE_FILES})

#add_executable(${TMP_EXECUTABLE} ${TMP_SRC_FILES} ${TMP_INCLUDE_FILES})
add_executable(${TAG_EXECUTABLE} ${TAG_SRC_FILES} ${TAG_INCLUDE_FILES})
add_executable(${ANCHOR_EXECUTABLE} ${ANCHOR_SRC_FILES} ${ANCHOR_INCLUDE_FILES})
add_executable(${ID_EXECUTABLE} ${ID_SRC_FILES} ${ID_INCLUDE_FILES})
add_executable(${MONITOR_EXECUTABLE} ${MONITOR_SRC_FILES} ${MONITOR_INCLUDE_FILES})

#target_link_libraries(${TMP_EXECUTABLE} PUBLIC rodos::rodos stdc++)
target_link_libraries(${TAG_EXECUTABLE} PUBLIC rodos::rodos stdc++)
target_link_libraries(${ANCHOR_EXECUTABLE} PUBLIC rodos::rodos stdc++)
target_link_libraries(${ID_EXECUTABLE} PUBLIC rodos::rodos stdc++)
target_link_libraries(${MONITOR_EXECUTABLE} PUBLIC rodos::rodos stdc++)

#target_include_directories(${EXECUTABLE_NAME} PRIVATE decadriver wrapper Inc)

#target_include_directories(${TMP_EXECUTABLE} PRIVATE decadriver wrapper ros_lib include)
target_include_directories(${TAG_EXECUTABLE} PRIVATE decadriver wrapper ros_lib include)
target_include_directories(${ANCHOR_EXECUTABLE} PRIVATE decadriver wrapper ros_lib include)
target_include_directories(${ID_EXECUTABLE} PRIVATE decadriver wrapper ros_lib include)
target_include_directories(${MONITOR_EXECUTABLE} PRIVATE decadriver wrapper ros_lib include)

#add_custom_command(
#    TARGET ${TMP_EXECUTABLE}
#    POST_BUILD
#    COMMAND ${CMAKE_OBJCOPY} ARGS -S -O binary ${TMP_EXECUTABLE} ${TMP_EXECUTABLE}.bin
#)

add_custom_command(
    TARGET ${TAG_EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -S -O binary ${TAG_EXECUTABLE} ${TAG_EXECUTABLE}.bin
)

add_custom_command(
    TARGET ${ANCHOR_EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -S -O  binary ${ANCHOR_EXECUTABLE} ${ANCHOR_EXECUTABLE}.bin
)

add_custom_command(
    TARGET ${ID_EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -S -O  binary ${ID_EXECUTABLE} ${ID_EXECUTABLE}.bin
)

add_custom_command(
    TARGET ${MONITOR_EXECUTABLE}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -S -O  binary ${MONITOR_EXECUTABLE} ${MONITOR_EXECUTABLE}.bin
)
