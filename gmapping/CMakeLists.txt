cmake_minimum_required(VERSION 2.8)
project(gmapping)

find_package(catkin REQUIRED nav_msgs nodelet openslam_gmapping roscpp tf rosbag_storage)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(slam_gmapping ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_gmapping ${catkin_EXPORTED_TARGETS})
endif()

add_library(slam_gmapping_nodelet src/slam_gmapping.cpp src/nodelet.cpp)
target_link_libraries(slam_gmapping_nodelet ${catkin_LIBRARIES})

add_executable(slam_gmapping_replay src/slam_gmapping.cpp src/replay.cpp)
target_link_libraries(slam_gmapping_replay ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
add_dependencies(slam_gmapping_replay ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS slam_gmapping slam_gmapping_replay
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  if(TARGET tests)
    add_executable(gmapping-rtest EXCLUDE_FROM_ALL test/rtest.cpp)
    target_link_libraries(gmapping-rtest ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    add_dependencies(tests gmapping-rtest)
  endif()

  # Need to make the tests more robust; currently the output map can differ
  # substantially between runs.
  catkin_download_test_data(
    ${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/gmapping/basic_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 322a0014f47bcfbb0ad16a317738b0dc)
  catkin_download_test_data(
    ${PROJECT_NAME}_hallway_slow_2011-03-04-21-41-33.bag
    http://download.ros.org/data/gmapping/hallway_slow_2011-03-04-21-41-33.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e772b89713693adc610f4c5b96f5dc03)
  catkin_download_test_data(
    ${PROJECT_NAME}_basic_localization_stage_groundtruth.pgm
    http://download.ros.org/data/gmapping/basic_localization_stage_groundtruth.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 abf208f721053915145215b18c98f9b3)
  catkin_download_test_data(
    ${PROJECT_NAME}_test_replay_crash.bag
    https://github.com/ros-perception/slam_gmapping_test_data/raw/master/test_replay_crash.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 bb0e086207eb4fccf0b13d3406f610a1)
  catkin_download_test_data(
    ${PROJECT_NAME}_test_turtlebot.bag
    https://github.com/ros-perception/slam_gmapping_test_data/raw/master/test_turtlebot.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 402e1e5f7c00445d2a446e58e3151830)
  catkin_download_test_data(
    ${PROJECT_NAME}_test_upside_down.bag
    https://github.com/ros-perception/slam_gmapping_test_data/raw/master/test_upside_down.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 3b026a2144ec14f3fdf218b5c7077d54)
  set(LOCAL_DEPENDENCIES gmapping-rtest ${PROJECT_NAME}_basic_localization_stage_indexed.bag
     ${PROJECT_NAME}_test_replay_crash.bag
     ${PROJECT_NAME}_test_turtlebot.bag
     ${PROJECT_NAME}_test_upside_down.bag
     ${PROJECT_NAME}_hallway_slow_2011-03-04-21-41-33.bag
     ${PROJECT_NAME}_basic_localization_stage_groundtruth.pgm
     slam_gmapping
     slam_gmapping_replay
  )
  add_rostest(test/basic_localization_stage.launch DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_stage_replay.launch DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_stage_replay2.launch DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_symmetry.launch DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_upside_down.launch DEPENDENCIES ${LOCAL_DEPENDENCIES})
  add_rostest(test/basic_localization_laser_different_beamcount.test DEPENDENCIES ${LOCAL_DEPENDENCIES})
endif()
