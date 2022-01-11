mkdir ./include/
cd ./include/

# install OpenCV
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build
cd build
cmake ..
cmake --build .
make -j4
sudo make install
sudo ldconfig
cd ../../
