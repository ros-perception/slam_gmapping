cmake_minimum_required(VERSION 2.8)
project(slam_gmapping)

find_package(catkin REQUIRED gmapping roscpp rosconsole tf)

catkin_package()

find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})


message(INFO ${CATKIN_INCLUDE_DIRS})

include_directories(include include/gmapping)
add_executable(slam_gmapping src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(slam_gmapping gridfastslam sensor_odometry sensor_range utils scanmatcher)
target_link_libraries(slam_gmapping ${BOOST_LIBRARIES} ${catkin_LIBRARIES})

catkin_add_gtest(rtest test/rtest.cpp)

# Need to make the tests more robust; currently the output map can differ
# substantially between runs.
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_indexed.bag test/basic_localization_stage_indexed.bag 322a0014f47bcfbb0ad16a317738b0dc)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/hallway_slow_2011-03-04-21-41-33.bag test/hallway_slow_2011-03-04-21-41-33.bag e772b89713693adc610f4c5b96f5dc03)
##rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_groundtruth.pgm test/basic_localization_stage_groundtruth.pgm abf208f721053915145215b18c98f9b3)
#rosbuild_add_rostest(test/basic_localization_stage.launch)
#rosbuild_add_rostest(test/basic_localization_laser_different_beamcount.test)
