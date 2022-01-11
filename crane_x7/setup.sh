mkdir ./include/
cd ./include/

# DynamixelSDK
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ./DynamixelSDK/c++/build/linux64/
make
cd ../../../../

# OpenCV
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build & cd build
cmake ..
cmake --build .
make -j4
sudo make install
sudo ldconfig
cd ../../

# librealsense
sudo apt install libssl-dev libusb-1.0.0-dev pkg-config libgtk-3-dev pkg-config libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
git clone https://github.com/IntelRealSense/librealsense
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-ubuntu-kernel-ubuntu-lts.sh
mkdir build & cd build
cmake .. -DBUILD_EXAMPLES=true
make
sudo make install
sudo ldconfig
cd ../../
