echo "Building ORB_SLAM3 Examples ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=ON
make -j4
ldconfig