#include <cstdio>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include "vl53l5cx_api.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>

class JetsonAvoidance :  public rclcpp::Node 
{

  public:
  explicit JetsonAvoidance() : Node("jetson_obs_distance")
  {
    rmw_qos_profile_t sens_qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(sens_qos_profile.history, 5), sens_qos_profile);

    // Fill the publisher
    _obs_distance_msg = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);

    _sensor_time_start = std::chrono::steady_clock::now();
    
    // Configure the VL53L5CX
    SensorInit();

    _timer = this->create_wall_timer(std::chrono::milliseconds(15), std::bind(&JetsonAvoidance::DistanceRead, this));
  }

  private:

    void SensorInit(void);
    void DistanceRead(void);
    // Create publisher
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _obs_distance_msg;
    
    // Create Timer
    rclcpp::TimerBase::SharedPtr _timer;
    
    // Timestamp of the node starting
    std::chrono::_V2::steady_clock::time_point _sensor_time_start; 
    // VL53L5CX configuration & results struct
    VL53L5CX_Configuration _config;
    
    VL53L5CX_ResultsData _results;

    uint8_t _return_status;
    uint8_t _is_active;
    uint8_t _data_ready;
    uint8_t _ranging_started{0};
};

void JetsonAvoidance::SensorInit()
{

  while(!_ranging_started) {
    _return_status = vl53l5cx_comms_init(&_config.platform, 1);

      if(_return_status != 0) {

        RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX device not connected");

      } else {
        _return_status = vl53l5cx_is_alive(&_config, &_is_active);
        
        if(!_is_active || (_return_status != 0)) {
          RCLCPP_WARN(this->get_logger(), "WARNING: VL53L5CX device not alive");
        }

        _return_status = vl53l5cx_init(&_config);
	      
	      if(_return_status != 0){

          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX device init failed");
	      }
        _return_status = vl53l5cx_set_resolution(&_config, VL53L5CX_RESOLUTION_8X8);

        if(_return_status != 0) {
          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX resolution not set");
        } else {
          _return_status = vl53l5cx_set_ranging_frequency_hz(&_config, 30);
          _return_status = vl53l5cx_start_ranging(&_config);
          vl53l5cx_check_data_ready(&_config, &_data_ready);
          RCLCPP_INFO(this->get_logger(), "VL53L5CX checking data ready");
      	  _ranging_started = 1;
        }
      }
    }

}

void JetsonAvoidance::DistanceRead() {

  uint8_t distance_indx = 0;
  px4_msgs::msg::ObstacleDistance obs;
  auto sensor_time_end = std::chrono::steady_clock::now();

  vl53l5cx_check_data_ready(&_config, &_data_ready);
  
  RCLCPP_INFO(this->get_logger(), "VL53L5CX reading results...");
  if(_data_ready) {
    vl53l5cx_get_ranging_data(&_config, &_results);
    // Extract the middle row since this is a 3D rangefinder, but PX4 obstacle dist array is 2D
    for(int index = 39; index > 31; --index) {

      // The receiver array index 0 starts at bottom left, but image is flipped vertically and horizontally
      distance_indx = 39 - index;

      uint16_t distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
      obs.distances[distance_indx] = distance_val;
    }

    obs.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(sensor_time_end - _sensor_time_start).count();
    
    // In 8x8 ranging mode the FoV is 60 degrees
    obs.angle_offset = 31.5f;
    obs.increment = 7.9f;
    
    // Sensor limits in centimeters
    obs.min_distance = 2;
    obs.max_distance = 400;
    _obs_distance_msg->publish(obs);

  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetsonAvoidance>());
  rclcpp::shutdown();
  return 0;
}
