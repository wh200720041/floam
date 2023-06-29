# CMake
sudo apt-get install cmake -y
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev -y
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev -y
# Eigen3
sudo apt-get install libeigen3-dev -y
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev -y

cd ../../

git clone https://ceres-solver.googlesource.com/ceres-solver

cd ceres-solver

mkdir ceres-bin

cd ceres-bin

cmake ..

make -j8

sudo make install