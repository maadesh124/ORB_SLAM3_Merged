## Installation guide for ORB-SLAM3 on UBUNTU 
### Install Systemwide dependencies
```
sudo apt update

sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev

sudo apt-get install libglew-dev libboost-all-dev libssl-dev

sudo apt install libeigen3-dev
```
if 'package not found' errors occur for python-dev, libjasper, libopenjpeg use following command
```
# occurs while installing system wide dependencies
sudo apt-get install python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-dev libjasper-dev
```

### Install OpenCV 4.6.0

```
cd ~
mkdir Dev && cd Dev
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.6.0
```

```
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make
sudo make install
```

### Install Pangolin
```
cd ~/Dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
mkdir build 
cd build 
cmake .. -D CMAKE_BUILD_TYPE=Release 
make
sudo make install
```

if libpangolin_windowing.so.0 not found use the following command
```
export LD_LIBRARY_PATH=/home/stemtec/Dev/Pangolin/build/:$LD_LIBRARY_PATH
```

### Download ORB_SLAM3
```
cd ~/Dev
git clone https://github.com/maadesh124/ORB_SLAM3_Merged.git
cd ORB_SLAM3_Merged
```

if the system does not find libepoxy use the following command
```
# occurs while compiling orblam3
sudo apt-get install libepoxy-dev
```

### Install Thirdparty dependencies and Vocabulary
```
bash ./Thirdparty.sh
```

### Compile ORB_SLAM_3(excluding dependencies ,they are already compiled)
```
bash ./build.sh
```

### Download Euroc Dataset
```
cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```

### Run Simulation
```
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono
```
### Download and Run VIT Dataset
```
# Download
cd ~/Datasets
wget --no-check-certificate \
     "https://drive.usercontent.google.com/download?id=1BEyUl_TtRHgjUZ_B2MTyC4ye4exnmJaR&confirm=t" \
     -O "VITDATA.zip"
unzip VITDATA.zip -d VITDATA

# Run
cd ~/Dev/ORB_SLAM3_Merged
./Examples/Monocular/mono_euroc  ./Vocabulary/ORBvoc.txt ~/Dev/ORB_SLAM3_Merged/Examples/Monocular/vitcam.yaml ~/Datasets/VITDATA ~/Datasets/VITDATA/mav0/cam0/timestamps.txt dataset-MH01_monoi
```

### Custom Data Preprocessing
```
cd ~/Dev/ORB_SLAM3_Merged/Preprocess
python3 to_euroc.py ~/Downloads/sample.mp4 --output ~/Datasets/c1
```

### Camera Calibration
```
cd ~/Dev/ORB_SLAM3_Merged/Calibration
python3 parameters.py ./sample_data
```

### Run custom data
```
./Examples/Monocular/mono_euroc  ./Vocabulary/ORBvoc.txt ~/Dev/ORB_SLAM3_Merged/Examples/Monocular/vitcam.yaml ~/Datasets/c1 ~/Datasets/c1/mav0/cam0/timestamps.txt dataset-MH01_monoi

```






