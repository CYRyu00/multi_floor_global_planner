# CMake generated Testfile for 
# Source directory: /home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description
# Build directory: /home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_jackal_description_roslaunch-check_launch_description.launch "/home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/build/test_results/jackal_description/roslaunch-check_launch_description.launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/build/test_results/jackal_description" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/build/test_results/jackal_description/roslaunch-check_launch_description.launch.xml\" \"/home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/launch/description.launch\" ")
set_tests_properties(_ctest_jackal_description_roslaunch-check_launch_description.launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/CMakeLists.txt;8;roslaunch_add_file_check;/home/ryu/snu/UROP/catkin_ws/src/jackal/jackal_description/CMakeLists.txt;0;")
subdirs("gtest")
