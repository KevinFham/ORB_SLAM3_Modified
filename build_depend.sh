echo "Configuring and building Thirdparty/opencv4.4.0"

cd Thirdparty/opencv4.4.0
mkdir build
cd build
cmake .. -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_FFMPEG=ON
make -j4
make install
ldconfig

cd ../../Pangolin

echo "Configuring and building Thirdparty/Pangolin"

mkdir build
cd build
cmake ..
cmake --build .
make -j4
make install

cd ../../DBoW2

echo "Configuring and building Thirdparty/DBoW2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

chmod -R 777 ./*

