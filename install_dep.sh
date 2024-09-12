apt-get install -y cmake wget build-essential libgl1-mesa-dev
sudo apt-get install -y libx11-*
sudo apt-get install -y libx11*

sudo apt-get install -y libxcb-*
sudo apt-get install -y libxcb*

sudo apt-get install -y libxkbcommon-devsudo
sudo apt-get install -y libxkbcommon-x11-dev

sudo apt install fonts-noto-cjk

sudo apt-get install -y libyaml-cpp-dev

sudo apt install -y locales

sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

sudo apt-get install git cmake

echo "export LANG=en_US.UTF-8" >> ~/.bashrc
echo "export LC_ALL=en_US.UTF-8" >> ~/.bashrc

source ~/.bashrc

mkdir 3rdParty
cd 3rdParty

wget https://download.qt.io/archive/qt/6.5/6.5.3/single/qt-everywhere-src-6.5.3.tar.xz
tar -xf qt-everywhere-src-6.5.3.tar.xz
rm qt-everywhere-src-6.5.3.tar.xz
cd qt-everywhere-src-6.5.3
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../../Qt6 \
 -DFEATURE_xcb=ON \
 -DFEATURE_xcb-xlib=ON \
 -DFEATURE_xcb-sync=ON \
 -DFEATURE_xcb-randr=ON \
 -DFEATURE_xcb-render=ON \
 -DFEATURE_xcb-shape=ON \
 -DFEATURE_xcb-xinerama=ON \
 -DFEATURE_xkbcommon-x11=ON

# sudo apt-get install qt6-base-dev

make -j6 && make install
cd ../..
rm -r qt-everywhere-src-6.5.3

wget https://github.com/g-truc/glm/archive/refs/tags/1.0.1.tar.gz
tar -xf 1.0.1.tar.gz
rm 1.0.1.tar.gz 1.0.1.tar.gz.1





