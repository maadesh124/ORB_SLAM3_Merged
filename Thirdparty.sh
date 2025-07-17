
#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Cloning Thirdparty and Vocabulary ..."
git clone https://github.com/maadesh124/Thirdparty_ORB_SLAM3.git "Thirdparty"

echo "Extracting Vocabulary ..."
mkdir -p Vocabulary
mv Thirdparty/ORBvoc.txt.tar.gz Vocabulary
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

rm -rf Thirdparty/.git

echo "Configuring and building Thirdparty/DBoW2 ..."

cd "Thirdparty/DBoW2"
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

