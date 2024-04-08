# camera lidar calibration

Below are the steps to get the Extrinsic matrix between the lidar point clouds and the camera images. For the calibration, we need the dataset in the below format:

1. To get the camera intrinsics -> minimum 25 checkerboard pattern images (preferred inner row x col number = 8 x 6 ; rectangular frame is preferred because it helps detect 180 degree rotations ; row x col must be odd x even or vice versa for MATLAB and opencv)

2. Rosbags with lidar and camera data of the same FOV at the same timeframe. The lidar we have is a Livox mid-360 and the camera used is mapir RGN wide.

## Install dependencies

[PCL installation](https://pointclouds.org/downloads/)
[Eigen installation](https://robots.uc3m.es/installation-guides/install-eigen.html)
[Ceres-solver installation](http://ceres-solver.org/installation.html)
Cmake
OpenCV
Pangolin

Below are the 2 main repositories that were used for this calibration:

[repo1](https://github.com/DoongLi/livox_camera_lidar_calibration_d2) ; also clone the [repo1_depend](https://github.com/Livox-SDK/livox_ros_driver2) repo inside the src folder of repo 1 and then build it since the mid-360 used livox-ros driver2.

```bash
git clone https://github.com/DoongLi/livox_camera_lidar_calibration_d2
cd ..
catkin_make
source devel/setup.bash
```
[repo2](https://github.com/PJLab-ADG/SensorsCalibration)

## step1: Get the camera intrinsics

We can use different ways to get the camera intrinsics:
I.  MATLAB (oddxeven row column - inbuilt camera claibrator app) 

II. OpenCV (oddxeven row column - Use the code here: [click me](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html))

III.Using repo 1 (checker board size : inner row x col = 8 x 6) : 
Configure the corresponding path and parameters in cameraCalib.launch after getting the photo data. The default path is to put the photo data under data/camera/photos, and write the corresponding photo names in data/camera/in.txt. Then run the command to start calibrating:

`roslaunch camera_lidar_calibration cameraCalib.launch`

Calibration results will be saved in data/camera/result.txt, including reprojection errors, intrinsic parameters and distortion correction parameters

IV. Using repo 2 (For checker board size and instructions follow this: [click me](https://github.com/PJLab-ADG/SensorsCalibration/tree/master/camera_intrinsic))

Note: Board size varies in each method and is very important in calculating the intrinsics.
For this project, below intrinsic parameters were used:
```bash
image_width: 640
image_height: 480
camera_name: mapir_rgn
camera_matrix:
  rows: 3
  cols: 3
  data: 462.2446290114162, 0, 308.8115146027777, 0, 458.4751339488662, 249.0395565896586, 0, 0, 1
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: 0.03527823763959915, -0.06383912241357363, 0.007655239948373614, 0.008907391781204871, 0
```

## step2: convert lidar data to pcd

In the repo1 : Check the rosbag path in pcdTransfer.launch, set the number of rosbags, and name the rosbag 0.bag, 1.bag...etc.

Run the command to convert the rosbag to PCD files in batches, they will be saved in defaut path data/pcdFiles.
`roslaunch camera_lidar_calibration pcdTransfer.launch`

PCD files can also be viewed by:
`pcl_viewer -use_point_picking xx.pcd`

## step3: Get the extrinsic matrix between the corresponding lidar-pcd and camera-images

Using the above pcd files and the corresponding images in repo2:
```bash
cd lidar2camera
mkdir -p build && cd build
cmake .. && make
```
Using the manual calibration tool:

```bash
cd ~./manual_calib/
./bin/run_lidar2camera <image_path> <pcd_path> <intrinsic_json> <extrinsic_json>
```

The calibration panel opens where we can manually align the points to the image and once they are aligned, click the save buttin to save the reult. The results (calibrated image, extrinsic and intrinsic matrix) are stored by default at running directory `~./manual_calib/` :
 ```bash
 Extrinsic:
R:
0.0121008 -0.999863 -0.0112902
0.0133341 0.0114512 -0.999846
0.999838 0.0119484 0.0134707
t: 0.0134153 -0.352602 -0.575013

Intrinsic:
2120.29 0 949.828
0 2071.72 576.237
0 0 1
************* json format *************
Extrinsic:
[0.0121008,-0.999863,-0.0112902,0.0134153],[0.0133341,0.0114512,-0.999846,-0.352602],[0.999838,0.0119484,0.0134707,-0.575013],[0,0,0,1]

Intrinsic:
[2120.29,0,949.828],[0,2071.72,576.237],[0,0,1]

Distortion:
[-0.108145,0.138668,-0.00379757,-0.00484127]
 ```

Refer this github link for more information: [click me](https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2camera)

Note: The Lidar-camera calibration can also be done using just the repo1 where we manually click the board corners in both the images and the pcds and save these corners. Then we have to run the `getExt1.launch` file to get the extrinsic between the camera and the lidar. But there seems to be some convergence problem in the script. So, the cloud points were not aligned with the image as expected.



