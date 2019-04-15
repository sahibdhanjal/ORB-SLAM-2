export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/sahib/Documents/ORB_SLAM2/Examples/ROS

echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
