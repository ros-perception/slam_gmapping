^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openslam_gmapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2019-07-10)
------------------
* CMakeLists.txt: fix install command (install libs, install includes to corrrect position as well as hxx (`#30 <https://github.com/ros-perception/openslam_gmapping/issues/30>`_)
* Contributors: Kei Okada

0.2.0 (2019-07-07)
------------------
* update license to BSD-3 (`#29 <https://github.com/ros-perception/openslam_gmapping/issues/29>`_)
  
  * update maintainer to ROS Orphaned Package Maintainers
  * Cleanup of cmake and package.xml
  * remove compile error
  /home/user/gmapping_ws/src/openslam_gmapping/gridfastslam/gfs2rec.cpp: In member function ‘virtual void ResampleRecord::read(std::istream&)’:
  /home/user/gmapping_ws/src/openslam_gmapping/gridfastslam/gfs2rec.cpp:148:17: error: redeclaration of ‘unsigned int i’
  unsigned int i;
  ^
  /home/user/gmapping_ws/src/openslam_gmapping/gridfastslam/gfs2rec.cpp:147:21: error: ‘unsigned int i’ previously declared here
  for (unsigned int i=0; i< dim; i++){
  ^
  make[2]: *** [CMakeFiles/gfs2rec.dir/gridfastslam/gfs2rec.cpp.o] Error 1
  make[1]: *** [CMakeFiles/gfs2rec.dir/all] Error 2
  make[1]: *** Waiting for unfinished jobs....
  * update for NANs, comply with REP117
  * apply missing patches
  * update cpp header location in #include directive
  for fullfile in $(cd include/; find gmapping/ -type f -print); do headerfile=$(basename $fullfile); echo $fullfile; for targetfile in $(find . -type f -not -path "*/.git/*" -print); do sed -i "s@\([\"<]\)$headerfile\([>\"]\)@\1$fullfile\2@g" $targetfile | grep $headerfile; done; done
  for fullfile in $(cd include/; find gmapping/ -type f -print); do headerfile=$(basename $fullfile); for targetfile in $(find . -type f -not -path "*/.git/*" -print); do sed -i "s@\([\"<]\)[a-z0-9\_/]*/$headerfile\([>\"]\)@\1$fullfile\2@g" $targetfile | grep $headerfile; done; done
  * move cpp header fiels to include directory, by
  find . -iname *.h* -print -exec bash -c 'file={}; dir=; mkdir -p include/gmapping//; git mv  include/gmapping//' \;
  * catkinize package CMakeLists.txt package.xml

* Forked from original openslam_gmapping package (https://github.com/OpenSLAM-org/openslam_gmapping/tree/79ef0b0e6d9a12d6390ae64c4c00d37d776abefb  
* Contributors: Kei Okada, Michael Ferguson, Mike Ferguson, William Woodall, grisetti, stachnis
