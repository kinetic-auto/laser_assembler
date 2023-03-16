# CMake generated Testfile for 
# Source directory: /home/sander/ros/src/laser_assembler
# Build directory: /home/sander/ros/src/laser_assembler/cmake-build-release
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_laser_assembler_rostest_test_test_laser_assembler.launch "/home/sander/ros/src/laser_assembler/cmake-build-release/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sander/ros/src/laser_assembler/cmake-build-release/test_results/laser_assembler/rostest-test_test_laser_assembler.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sander/ros/src/laser_assembler --package=laser_assembler --results-filename test_test_laser_assembler.xml --results-base-dir \"/home/sander/ros/src/laser_assembler/cmake-build-release/test_results\" /home/sander/ros/src/laser_assembler/test/test_laser_assembler.launch ")
set_tests_properties(_ctest_laser_assembler_rostest_test_test_laser_assembler.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/sander/ros/src/laser_assembler/CMakeLists.txt;81;add_rostest;/home/sander/ros/src/laser_assembler/CMakeLists.txt;0;")
subdirs("gtest")
