mkdir build
cd build&&
cmake ..&&
make -j4&&
cd ..&&export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib && ./bin/main
