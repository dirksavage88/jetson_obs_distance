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
    this->declare_parameter("starting_gpio", 12);

    this->declare_parameter("i2c_bus", 1); 
    _num_sensors = this->get_parameter("num_sensors").as_int();
    _i2c_bus = this->get_parameter("i2c_bus").as_int();
    // We start at a gpio number and iterate until num_sensors 
    // E.g. gpio 12 -> sensor 0, gpio++ for each sensor iterated
    _gpio_index = this->get_parameter("starting_gpio").as_int();
    SetAddress(static_cast<char>(_i2c_bus));
  }
	
  private:

    // Need 6 sensors with approximately 60 degree fov to get 360, plus upward and downard sensors, total of 8
    VL53L5CX_Configuration Sensor[MAX_SENSORS];
    int _num_sensors{1};
    unsigned char _sensor_address{0x29};
    int _i2c_bus{1};
    int SetAddress(char bus);
    int _gpio_index{0};
    int _gpio_max{0};
    void Vl53l5cxReset(int gpio, bool reset);
};


int Vl53l5cxAddress::SetAddress(char bus) {
	
	
	_gpio_max = _num_sensors + _gpio_index;
	int selected_gpio = 0;
	for(int i = 0; i < _num_sensors; i++){
                //we want to set 12, so 13, 14, 15 pulled low
                selected_gpio = i + _gpio_index;

		for (int gpio_num = _gpio_index; gpio_num < _gpio_max; ++gpio_num) {
			if(gpio_num == selected_gpio) {
				Vl53l5cxReset(gpio_num, false);
			}

			else {
				Vl53l5cxReset(gpio_num, true);
			}
		}		
		if(vl53l5cx_comms_init(&Sensor[i].platform, bus, DEFAULT_ADDRESS)){
			printf("VL53L5CX comms init failed"); /*%d\n", gpio_pin[i]);*/
			exit(-1);
		}
		uint16_t new_address = _sensor_address + i + 1;
		// 7-bit i2c address
		if(vl53l5cx_set_i2c_address(&Sensor[i], new_address<<1)){
			printf("VL53L5CX address could not change (%d)\n",i);
			exit(-1);
		}
		// What is this used for?
		Sensor[i].platform.address= new_address;
		vl53l5cx_comms_close(&Sensor[i].platform);
	}

	// All i2c addresses configured; pull all sensor resets up
	
	for (int sensor = _gpio_index; sensor < _gpio_max; ++sensor) {
		Vl53l5cxReset(sensor, false);
	}		

	return 1;
}

void Vl53l5cxAddress::Vl53l5cxReset(int gpio_index, bool reset) {
	char direction_path[35];
	char value_path[30];
	std::string s_gpio = std::to_string(gpio_index);
	
	// Export the gpio
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		perror("Error opening sys class");
		exit(1);
	}
	
	if(write(fd, s_gpio.c_str(), s_gpio.size()) < 0) {

		perror("Error exporting gpio to sys class");
		exit(1);
	}
	close(fd);

	// Set the gpio direction out
	snprintf(direction_path, sizeof(direction_path), "/sys/class/gpio/gpio%d/direction", gpio_index);
	fd = open(direction_path, O_WRONLY);

	if (fd == -1) {
	
		perror("Error opening gpio direction");
		exit(1);
	}

	if(write(fd, "out", 3) != 3) {
	
		perror("Error setting gpio direction");
		exit(1);
	
	}
	close(fd);

	// Pull pin high/low
	snprintf(value_path, sizeof(value_path), "/sys/class/gpio/gpio%d/value", gpio_index);
	fd = open(value_path, O_WRONLY);

	if (fd == -1) {
	
		perror("Error opening gpio value");
		exit(1);
	}
	if(reset) {
	
		if(write(fd, "0", 1) != 1) {

			perror("Error writing to sys class");
			exit(1);
		}
	
	}
	else {
	
		if(write(fd, "1", 1) != 1) {

			perror("Error writing to sys class");
			exit(1);
		}
	}
	close(fd);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vl53l5cxAddress>());
  rclcpp::shutdown();
  return 0;
}
