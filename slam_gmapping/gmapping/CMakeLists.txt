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

# Need to make the tests more robust; currently the output map can differ
# substantially between runs.
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage.bag test/basic_localization_stage.bag)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/gmapping/basic_localization_stage_groundtruth.pgm test/basic_localization_stage_groundtruth.pgm)
#rosbuild_add_rostest(test/basic_localization_stage.launch)
