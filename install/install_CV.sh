echo "Configuring and installing OpenCV ..."

OPENCV_VERSION='4.1.0'
sudo apt-get -y update

# Install Dependencies
sudo apt-get install -y build-essential cmake
sudo apt-get install -y qt5-default libvtk6-dev
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev
sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev
sudo apt-get install -y libtbb-dev libeigen3-dev
sudo apt-get install -y ant default-jdk
sudo apt-get install -y unzip wget

cd /tmp/

# Clone Contrib Files
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout ${OPENCV_VERSION}
cd ../

# Install Library
wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
unzip ${OPENCV_VERSION}.zip
rm ${OPENCV_VERSION}.zip
mv opencv-${OPENCV_VERSION} OpenCV
cd OpenCV
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_opencv_python=OFF \
    -D BUILD_opencv_cudacodec=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
    -D WITH_CUDA=OFF \
    -D ENABLE_FAST_MATH=1 \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D WITH_CUBLAS=1 \
    -D ENABLE_PRECOMPILED_HEADERS=OFF ..

make -j8
sudo make install
sudo ldconfig