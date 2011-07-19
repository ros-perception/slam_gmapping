cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(gmapping)

# Build GMapping
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.gmapping
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of GMapping failed")
endif(_make_failed)

include_directories(include include/gmapping)
link_directories(${PROJECT_SOURCE_DIR}/lib)
rosbuild_add_executable(bin/slam_gmapping src/slam_gmapping.cpp src/main.cpp)
target_link_libraries(bin/slam_gmapping gridfastslam sensor_odometry sensor_range utils scanmatcher)
#rosbuild_add_executable(tftest src/tftest.cpp)

rosbuild_add_executable(test/rtest EXCLUDE_FROM_ALL test/rtest.cpp)
rosbuild_add_gtest_build_flags(test/rtest)
rosbuild_declare_test(test/rtest)

# Need to make the tests more robust; currently the output map can differ
# substantially between runs.
rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_indexed.bag test/basic_localization_stage_indexed.bag 322a0014f47bcfbb0ad16a317738b0dc)
rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/hallway_slow_2011-03-04-21-41-33.bag test/hallway_slow_2011-03-04-21-41-33.bag e772b89713693adc610f4c5b96f5dc03)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_groundtruth.pgm test/basic_localization_stage_groundtruth.pgm abf208f721053915145215b18c98f9b3)
rosbuild_add_rostest(test/basic_localization_stage.launch)
rosbuild_add_rostest(test/basic_localization_laser_different_beamcount.test)
