echo "Building ROS nodes"

cd Examples/ROS/orb_slam2_ros
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
