#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "vl53l5cx_api.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cstdio>

#define MAX_SENSORS 8
#define DEFAULT_ADDRESS 0x52
class Vl53l5cxAddress: public rclcpp::Node {
 
  public:
  explicit Vl53l5cxAddress() : Node("vl53_set_address") {
   
    this->declare_parameter("num_sensors", 1);
    //this->declare_parameter("starting_address", "0x29"); 
    this->declare_parameter("i2c_bus", 1); 
    _num_sensors = this->get_parameter("num_sensors").as_int();
    _i2c_bus = this->get_parameter("i2c_bus").as_int();
    SetAddress(static_cast<char>(_i2c_bus));
  }
	
  private:

    // Need 6 sensors with approximately 60 degree fov to get 360, plus upward and downard sensors, total of 8
    VL53L5CX_Configuration Sensor[MAX_SENSORS];
    int _num_sensors{1};
    unsigned char _sensor_address{0x29};
    int _i2c_bus{1};
    int SetAddress(char bus);
};


int Vl53l5cxAddress::SetAddress(char bus) {


	for(int i = 0; i < _num_sensors; i++){

		if(vl53l5cx_comms_init(&Sensor[i].platform, bus, DEFAULT_ADDRESS)){
			printf("VL53L5CX comms init failed"); /*%d\n", gpio_pin[i]);*/
			exit(-1);
		}
		if(vl53l5cx_set_i2c_address(&Sensor[i], (_sensor_address + i + 1)<<1)){
			//printf("VL53L5CX address could not change (%d)\n",i);
			exit(-1);
		}
		Sensor[i].platform.address= 0x30;
		vl53l5cx_comms_close(&Sensor[i].platform);
	}

	return 1;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vl53l5cxAddress>());
  rclcpp::shutdown();
  return 0;
}
