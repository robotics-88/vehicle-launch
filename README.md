# prototype vehicle
Note in either vehicle case, ROS master must be the PC. Thus before any of the below, launch `roscore` on the PC.

## RPi Drone

First ssh into the drone:

`ssh ubuntu@192.168.0.50`

Launch MAVROS on the drone. Note that the ROS Master has already been set to the host PC. This command works when Pixhawk connected to Pi by USB:

`roslaunch mavros apm.launch`

If MAPIR NIR camera is plugged into Pi, start MAPIR image publishing with:

`roslaunch vehicle_launch mapir_nir.launch`

### On PC

Launch all offboard perception with (remove last arg once Sequoia camera integrated):

`roslaunch vehicle_launch prototype_offboard.launch do_multispectral_viz:=false`

## Nano Drone

First ssh into the drone (showing static IP for NETGEAR88-5G):

`ssh ubuntu@192.168.1.15`

Launch all nodes with (default is ZED + Mapir, include last arg if also Attollo):

`roslaunch vehicle_launch prototype.launch use_attollo:=true`

### On PC

Launch vizualization with:

`roslaunch vehicle_launch viz_host_machine.launch`

## Cameras

At the moment, camera device IDs often change on start-up. Until we can automatically detect and assign IDs, check for camera IDs with:

`v4l2-ctl --list-devices`

Then update the `device_id` param in the respective launch files for Attollo and Mapir.

# recording

To record only the inputs (i.e., for a bag to be used in development), run:

`roslaunch vehicle_launch rosbag_record_inputs.launch record_prefix:=<your dir>`

^This should generally be preferred compared to below. Recording all topics risks overrunning the buffer significantly and losing all data.

To record all required variables, input or output (i.e., for a bag from flight testing for later analysis), run:

`roslaunch vehicle_launch rosbag_record_all_io.launch record_prefix:=<your dir>`

After flight, if the rosbag record terminal shows warning messages about the buffer overrun, stop the prototype ROS nodes before the bag, and give some time (at least 60sec) before stopping the rosbag node. Verify bags by:

```
cd ~/dev/bags
rosbag info <yourbag>.bag
```
Then compress all valid bags (bags with any topics shown) with `rosbag compress *.bag` after moving all valid bags to a subfolder. This will significantly speed up file transfer.

If jpegs are needed, run

`roslaunch vehicle_launch jpeg_from_bag.launch bag:=<your bag>`,

then `mv ~/.ros/frame*.jpg <desired dir>`.

# camera calibration

Requires a checkerboard. Generates the camera info yaml for undistorting images and 3D projection.

Example uses MAPIR camera. First, launch the camera ROS driver:

`roslaunch vehicle_launch mapir_nir.launch`

Then, run the camera calibrator:

`rosrun camera_calibration cameracalibrator.py --size 7x7 --square 0.05715 image:=/mapir_cam/image_raw camera:=/mapir_cam`

A more detailed tutorial for the calibrator can be found [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).