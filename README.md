# Jetson VL53L5CX obstacle distance companion sensor to PX4 autopilot. 

## Setup
Using the pololu VL53L5CX (can be powered via 5v directly, connect VIN, SCL, SDA, & ground to the jetson GPIO. 
![pololu_vl53l5](https://github.com/dirksavage88/jetson_obs_distance/assets/35986980/371272d0-727d-49e2-bbb6-c3c7d195433a)


Clone this repo into a ROS 2 overlay workspace alongside px4_msgs:
`colcon build --packages-select px4_msgs jetson_obs_distance`

Source the overlay:
`. install/setup.bash`

## Parameters

I2C bus index depends on the Jetson i2c port (and the jetson board used). Usually it is either bus 0, bus 1, or bus 7. For example on bus 0:
`ros2 run jetson_obs_distance jetson_obs_distance --ros-args -p i2c_bus:=0`

## Confirm obstacle distance message and the overlay in Qgroundcontrol

![collision_prevention](https://github.com/dirksavage88/jetson_obs_distance/assets/35986980/180f5361-1260-4227-9a1b-41ac2ce5e3a1)

## Testing
Built and tested on ROS 2 Foxy. Built package in ROS 2 humble without issues but not tested.

**Note that these sensors are for indoor use only! Outdoors the ambient light can be too strong for the VL53L5CX sensor.**
