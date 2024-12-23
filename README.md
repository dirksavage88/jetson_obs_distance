# Jetson VL53L5CX obstacle distance companion sensor to PX4 autopilot. 

## Setup
Using the pololu VL53L5CX (can be powered via 5v directly, connect VIN, SCL, SDA, & ground to the jetson 
![pololu_vl53l5](https://github.com/dirksavage88/jetson_obs_distance/assets/35986980/371272d0-727d-49e2-bbb6-c3c7d195433a)


Clone this repo into a ROS 2 overlay workspace alongside px4_msgs:
`colcon build --packages-select px4_msgs jetson_obs_distance`

Source the overlay:
`. install/setup.bash`

## Running the Nodes
There are a minimum 2 nodes in this package that need to be run in order to get obstacle distance map messages in PX4. 

The _jetson_obs_distance_ node runs the VL53L5CX driver and publishes to a laserscan message in ROS 2. 

_px4_obstacle_dist_ subscribes to the laserscan from one or more jetson_obs_distance instances and publishes them to a unified obstacle map in PX4. 

_vl53_set_address_ is required for setting the i2c address of multiple sensors in an array, but is optional if running only one sensor.

## Parameters


The default I2C bus index depends on the Jetson i2c port (and the jetson board used). Usually it is either bus 0, bus 1, or bus 7. For example on bus 0:
`ros2 run jetson_obs_distance jetson_obs_distance --ros-args -p i2c_bus:=0`

## Confirm obstacle distance message and the overlay in Qgroundcontrol

![obstacle_distance_map](https://github.com/user-attachments/assets/6b69ed63-d2a1-404d-942d-2cf82c5c5bf2)


## Changing the I2C address

In order to change the i2c address of the sensor, connect each _XSHUT_ to a different (but consecutive) GPIO on the jetson (E.g SPI1 gpio 12). Then run the _vl53_set_address_ node with the param _num_sensors_ set to the number of VL53L5CX devices attached to the bus. In the example of using gpio 12 (which is SPI1_MOSI on the nano) it will iterate _num_sensors_ (gpio 12, 13, 14, etc).

```
 gpio-12  (SPI1_MOSI           |sysfs               ) out hi    
 gpio-13  (SPI1_MISO           |sysfs               ) out hi    
 gpio-14  (SPI1_SCK            |sysfs               ) out hi    
 gpio-15  (SPI1_CS0            |sysfs               ) out hi    
```

## Testing
Built and tested on ROS 2 Foxy. Built package in ROS 2 humble without issues but not tested.

**Note that these sensors are for indoor use only! Outdoors the ambient light can be too strong for the VL53L5CX sensor.**
