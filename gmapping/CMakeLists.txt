cmake_minimum_required(VERSION 2.8)
project(gmapping)

find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp rostest tf)

find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(src)

catkin_package()

add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(slam_gmapping ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_gmapping ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS slam_gmapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
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
  add_rostest(test/basic_localization_stage.launch)
  add_rostest(test/basic_localization_laser_different_beamcount.test)
endif()
