echo "compiling only ORBSLAM3 (excluding dependencies)"
mkdir -p build

cd build
cmake ..
make

