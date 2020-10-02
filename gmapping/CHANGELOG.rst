^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gmapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2020-10-02)
------------------
* change find_package(Boost REQUIRED signales) to find_package(Boost REQUIRED) for noteic (`#84 <https://github.com/ros-perception/slam_gmapping/issues/84>`_)
* Contributors: Kei Okada

1.4.1 (2020-03-16)
------------------
* Merge pull request `#85 <https://github.com/ros-perception/slam_gmapping/issues/85>`_ from k-okada/install_nodelet
  install slam_gmapping_nodelet
* Merge pull request `#87 <https://github.com/ros-perception/slam_gmapping/issues/87>`_ from acxz/patch-1
  remove signals dep
* remove signals dep
  signals is included in boost > 1.70
* install slam_gmapping_nodelet
* Contributors: Kei Okada, Michael Ferguson, acxz

1.4.0 (2019-07-12)
------------------
* update license to BSD and maintainer to ros-orphaned-packages@googlegroups.com
  since original gmapping source and ROS openslam_gmapping package has been updated to the BSD-3 license, I think we have no reason to use CC for slam_gmapping package
* Contributors: Kei Okada

1.3.10 (2018-01-23)
-------------------
* Install nodelet plugin descriptor file. (`#56 <https://github.com/ros-perception/slam_gmapping/issues/56>`_)
* Contributors: Mikael Arguedas, gavanderhoorn

1.3.9 (2017-10-22)
------------------
* remove unused file
* add missing nodelet dependency to find_package
* make rostest in CMakeLists optional (`ros/rosdistro#3010 <https://github.com/ros/rosdistro/issues/3010>`_)
* Add nodelet implementation. (`#41 <https://github.com/ros-perception/slam_gmapping/issues/41>`_)
  * Add nodelet implementation.
  Add additional nodelet layer to mirror the node
  implementation. This allows the Slam GMapping
  library to be run as a nodelet instead. This
  would allow you to, for example, run it under
  the same nodelet manager as the nodelet producing
  the /scan output for greater efficiency.
  * Remove superfluous semicolons
  Removed superfluous semicolons and
  mildly clarified info stream output.
* fix comment, change type from double to int (`#40 <https://github.com/ros-perception/slam_gmapping/issues/40>`_)
  * fix comment, change type from double to int
  * fix comment, iterations param is not double but int
* Contributors: David Hodo, Kevin Wells, Lukas Bulwahn, Oscar Lima, Vincent Rabaud

1.3.8 (2015-07-31)
------------------
* fix a test that would take too long sometimes
* better verbosity
* add a test for upside down lasers
* add a test for symmetry
* make sure the laser sent to gmapping is always centered
* do not display warning message if scan is processed at some point
* Contributors: Vincent Rabaud

1.3.7 (2015-07-04)
------------------
* get replay to behave like live processing
* Contributors: Vincent Rabaud

1.3.6 (2015-06-26)
------------------
* Don't crash on exit from replay.
* replay: Add "on_done" command line parameter.
  Example usage:
  ros run gmapping slam_gmapping_replay --scan_topic=/scan --bag_filename=/tmp/in.bag --on_done "rosrun map_server map_saver -f foo" _particles:=100 _maxUrange:=10
* replay: Handle order-of-data-in-bag-file issues better at startup.
  Silently discard scans at startup that arrive prior to the TF frames
  required by those scans.
  TF can't interpolate from non-existant data, so discard scans that
  arrive with timestamps prior to the first TF frame.  We do this
  expediently by discarding laser scans that throw
  ExtrapolationException's until/unless we successfully process our first
  scan.
  Similarly, ignore LookupException's at startup.
* Fix indexing error for inverting scan.
* add test dependencies
* get replay test to pass
* rename test
* fixing include order
  removing pointer for Gmapping object
* Minor Fix thanks to vrabaud comments
* [new feature] replay on bag file
  The aim is to provide a way to get exactly the same map after running
  gmapping multiple times on the same rosbag file. It wasn't possible with the
  tool 'rosbag play', indeed one missing laser scan could provide really
  different results.
  Moreover, this modification allow to process rosbag offline at the maximum
  speed with the guarantee that all lasers scans are processed. It is
  useful in automatic tests and when finding optimal gmapping parameters with a script.
  Example usage:
  rosrun gmapping slam_gmapping_replay --scan_topic=/scan --bag_filename=/tmp/in.bag _particles:=100 _maxUrange:=10
  more options:
  rosrun gmapping slam_gmapping_replay --help
* spliting init and constructor
  The objective is to allow future handling of replay offline rosbag files.
  This commit also add a variable for the seed, with the objective is to allow to
  have repeateable run (for future offline rosbag replay)
* Added cfloat include
* Change arbitrary constant to FLT_EPSILON
* Added check that scan goes from -x to x
* Contributors: Laurent GEORGE, Patrick Doyle, Qiao Huang, Vincent Rabaud, Windel Bouwman

1.3.5 (2014-08-28)
------------------
* Fixed typo in slam_gmapping_pr2.launch
  Fixed a typo in the launchfile in the parameter "map_update_interval".
* Contributors: DaMalo

1.3.4 (2014-08-07)
------------------
* Reenabled temporal update in slam_gmapping.cpp
* Contributors: DaMalo

1.3.3 (2014-06-23)
------------------
* Adding the ability to set openslam_gmapping miminumScore through the ros parameter minimumScore
* Contributors: Koen Lekkerkerker, William Woodall

1.3.2 (2014-01-14)
------------------
* Contributors: Vincent Rabaud

1.3.1 (2014-01-13)
------------------
* Fix usage of scoped locks so that they are not immediately destroyed.
  fixes `#11 <https://github.com/ros-perception/slam_gmapping/issues/11>`_
* check for CATKIN_ENABLE_TESTING
* Contributors: Lukas Bulwahn, Stefan Kohlbrecher, William Woodall

1.3.0 (2013-06-28)
------------------
* Renamed to gmapping, adding metapackage for slam_gmapping
* catkinize slam_gmapping
* Changed reference frame from base to laser to account for upside down and/or back facing laserscanners.
  - Added a check if the scanner is facing down
  - Added a safety check if the scanner is aligned planar
  - Made laserscan min- and max-angles global as they are needed later for scanners with a negative angle-increment
  - Replaced the base->laser pose for gmapping with the identity transform and included the base->laser part into the gmap_pose
  - Removed a transform-lookup from the map->odom transformation process as it is not needed anymore
  These changes should make gmapping more robust against laserscanners that are mounted upside down, facing backwards or are rotating counter-clockwise.
  It will also allow gmapping to work with panning laserscanners, since the transform base->laser is no longer considered fixed.
* Fix poorly formed paths in patches
  These patches won't apply in Fedora because they contain "..", which is considered "unsafe"
* Fixed test files to use the new rosbag command layout.
* Respect tf_prefix when sending maps
* Fixed tf expiration
* Added tf_delay param
* Add gcc 4.7 patch and Precise support by removing wiped during installed
* Oneiric linker fixes, bump version to 1.2.6
* Convert to not use bullet datatypes directly
* Rejiggered linker lines to accommodate Oneiric's stricter linker behavior.
* Now uses angle_increment provided in laser scan message, instead of computing it myself (not sure why I was doing that, anyway), `#4730 <https://github.com/ros-perception/slam_gmapping/issues/4730>`_
* Applied patch to avoid assert when laser gives varying number of beams per
  scan, `#4856 <https://github.com/ros-perception/slam_gmapping/issues/4856>`_.  Added the bag from that ticket as a test case.
* Applied patch from `#4984 <https://github.com/ros-perception/slam_gmapping/issues/4984>`_ to fix occ grid generation with lasers that provide scans in reverse order
* Applied patch from `#4583 <https://github.com/ros-perception/slam_gmapping/issues/4583>`_ with misc fixes to our patch against gmapping
* Excluded test program from all build
* Applied typo fix from Maurice Fallon
* Added Ubuntu platform tags to manifest
* Removed unused inverted_laser parameter
* Added transform logic necessary to account for non-horizontal lasers. This
  change is intended to handle upside-down lasers, but should also work for
  non-planar lasers (as long as the vertical structure of the environment is
  continuous), `#3052 <https://github.com/ros-perception/slam_gmapping/issues/3052>`_. I tested minimally with a hacked version of Stage, but
  this functionality still needs to be validated on data from a real robot
  with an upside-down laser.
* Reindexed bag used in testing
* Added publication of entropy
* add entropy computation method
* Added occ_thresh parameter
* Turning time based updates off by default
* Updating so that gmapping updates on a timer when not moving. Added the temporalUpdate parameter and updated docs.
* Updated md5sums for new bags
* Threading publishing of transforms so that they are published regularly regardless of how long map updates take.
* Updated patch to fix gcc 4.4 warning, and made top-level Makefile call through to Makefile.gmapping on clean
* Updating to work with the navigation stack. Now publishes header information on map messages.
* Applied patch to update tf usage, `#3457 <https://github.com/ros-perception/slam_gmapping/issues/3457>`_
* Remove use of deprecated rosbuild macros
* Removed unused parameter
* Fix the position gmapping gives to the map's info.  Was trying to center the map on the origin, when it should just have been using the world positiong of the map's origin (`#3037 <https://github.com/ros-perception/slam_gmapping/issues/3037>`_)
* Added doc cleared to manifest
* Switched sleep to WallDuration, to avoid getting stuck after rosplay has run out of time data to publish
* Converted from tf::MessageNotifier to tf::MessageFilter.
* Reverted accidental change to CMakeLists.txt
* Added advertisement and publication of MapMetaData (docs are updated to
  match).  Added minimal test for the resulting map.  Updated local params to use
  NodeHandle("~").
* Added latched topic version of map, API cleared
* Updated manifest to explain version that we're using
* Remove ros/node.h inclusion
* tf publishes on new topic: \tf. See ticket `#2381 <https://github.com/ros-perception/slam_gmapping/issues/2381>`_
* Merging in changes from reorgnization of laser pipeline.
* removed redundant code (getOdomPose) that could result in unnecessary warnings
* Contributors: Ben Struss, Dave Hershberger, Dereck Wonnacott, Mike Ferguson, Scott K Logan, Vincent Rabaud, William Woodall, duhadway-bosch, eitan, gerkey, jfaust, jleibs, kwc, meeussen, vrabaud, wheeler
