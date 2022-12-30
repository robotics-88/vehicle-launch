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

First ssh into the drone (atm, only ethernet works):

`ssh ubuntu@192.168.0.17`

Launch all nodes with:

`roslaunch vehicle_launch prototype.launch`

### On PC

Launch vizualization with:

`roslaunch viz_host_machine.launch`

# recording

To record only the inputs (i.e., for a bag to be used in development), run:

`roslaunch vehicle_launch rosbag_record_inputs.launch record_prefix:=<your dir>`

To record all required variables, input or output (i.e., for a bag from flight testing for later analysis), run:

`roslaunch vehicle_launch rosbag_record_all_io.launch record_prefix:=<your dir>`