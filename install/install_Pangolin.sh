echo "Configuring and installing Pangolin ..."

cd /tmp/
git clone https://github.com/stevenlovegrove/Pangolin.git

cd Pangolin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
sudo make install