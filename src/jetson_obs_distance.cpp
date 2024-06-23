#include <cstdio>
#include <chrono>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include "vl53l5cx_api.hpp"
#include "jetson_obs_distance.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <px4_msgs/msg/timesync_status.hpp>

class JetsonAvoidance :  public rclcpp::Node 
{

  public:
  explicit JetsonAvoidance() : Node("jetson_obs_distance")
  {
    rmw_qos_profile_t sens_qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(sens_qos_profile.history, 5), sens_qos_profile);
    // Declare parameter for i2c bus on jetson (usually 0, 1, or 7)
    this->declare_parameter("i2c_bus", 1);

    // Fill the publisher
    _obs_distance_msg = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);

    _timesync_msg = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", qos, std::bind(&JetsonAvoidance::TimeSyncCB, this, std::placeholders::_1));
    
    // Always have a backup in case we aren't timesynced to PX4 time
    _sensor_time_start = std::chrono::steady_clock::now();
    
    // Get the i2c bus
    _i2c_bus = this->get_parameter("i2c_bus").as_int();
    // Configure the VL53L5CX
    SensorInit();

    // only create wall timer if sensor init was successful
     if(_return_status == 0) {
     
    	_timer = this->create_wall_timer(std::chrono::milliseconds(15), std::bind(&JetsonAvoidance::DistanceReadCB, this));
     }

  }

  private:

    void SensorInit(void);
    void DistanceReadCB(void);
    void TimeSyncCB(px4_msgs::msg::TimesyncStatus::UniquePtr msg);
    
    // Create publisher
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _obs_distance_msg;
    
    // Create subscriber to PX4 timesync messages
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_msg;
    
    // Create Backup Timer
    rclcpp::TimerBase::SharedPtr _timer;
    
    // Timestamp of the node starting
    std::chrono::_V2::steady_clock::time_point _sensor_time_start; 

    bool _synced;
    uint64_t _synced_ts;
    // VL53L5CX configuration & results struct
    VL53L5CX_Configuration _config;
    
    VL53L5CX_ResultsData _results;

    uint8_t _return_status{0};
    uint8_t _is_active{0};
    uint8_t _data_ready{0};
    uint8_t _ranging_started{0};
    uint8_t _init_retries{5};
    float _confidence_score{100};
    uint8_t _signal_qual[8];
    int _i2c_bus{1};
};

void JetsonAvoidance::SensorInit()
{
  SensorState sensor_status = SensorState::Uninitialized;

  while(sensor_status != SensorState::InitComplete) {

    switch(sensor_status) {

      case SensorState::Uninitialized:
        _return_status = vl53l5cx_comms_init(&_config.platform, static_cast<char>(_i2c_bus));

        if(_return_status != 0) {

          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX device not connected");
	        _return_status = 1;
	        // Exit and retry
          sensor_status = SensorState::Failure;
	        break;
	      }

	      sensor_status = SensorState::CommsInit;
	      break;

      case SensorState::CommsInit:
        _return_status = vl53l5cx_is_alive(&_config, &_is_active);
        
        if(!_is_active || (_return_status != 0)) {
          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX device not responding");
	        // Exit and retry
          sensor_status = SensorState::Failure;
          break;
        }

        sensor_status = SensorState::CheckAlive;
        break;

      case SensorState::CheckAlive:
        _return_status |= vl53l5cx_init(&_config);
	      
	      if(_return_status != 0){
          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX device init failed");
	        // Exit and retry
          sensor_status = SensorState::Failure;
          break;
	      }

	      sensor_status = SensorState::Init;
	      break;

      case SensorState::Init:
        _return_status |= vl53l5cx_set_resolution(&_config, VL53L5CX_RESOLUTION_8X8);

        if(_return_status != 0) {
          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX resolution not set");
	        // Exit and retry
          sensor_status = SensorState::Failure;
        }
        sensor_status = SensorState::SetRes;
        break;
      
      // Set the sensor resolution (8x8)
      case SensorState::SetRes:
        _return_status |= vl53l5cx_set_ranging_frequency_hz(&_config, 30);
        if(_return_status != 0) {
          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX frequency not set");
	        // Exit and retry
          sensor_status = SensorState::Failure;
          break;
        }
        sensor_status = SensorState::SetFreq;
        break;
      
      // Set the sensor frequency
      case SensorState::SetFreq:
        _return_status |= vl53l5cx_start_ranging(&_config);
        if(_return_status != 0) {
          RCLCPP_ERROR(this->get_logger(), "ERROR: VL53L5CX ranging failed");
	        // Exit and retry
          sensor_status = SensorState::Failure;
          break;
        }
        sensor_status = SensorState::RangingActive;
        break;

      case SensorState::RangingActive:
          vl53l5cx_check_data_ready(&_config, &_data_ready);
      	  _ranging_started = 1;
          sensor_status = SensorState::InitComplete;
          break;

      case SensorState::Failure:
	--_init_retries; 
	if(_init_retries == 0) {
          // Exit the main init loop
          return;
	}
	// Retry from the uninit state
	sensor_status = SensorState::Uninitialized;
	break;
      
      case SensorState::InitComplete:
        RCLCPP_INFO(this->get_logger(), "VL53L5CX Init Complete");
        break;

    }

  }

}

void JetsonAvoidance::TimeSyncCB(const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
  _synced_ts = msg->timestamp;
  _synced = true;  

}
void JetsonAvoidance::DistanceReadCB() {

  uint8_t distance_indx = 0;
  px4_msgs::msg::ObstacleDistance obs;
  auto sensor_time_end = std::chrono::steady_clock::now();

  vl53l5cx_check_data_ready(&_config, &_data_ready);
  
  // Collect ranging results array from the VL53L5CX
  if(_data_ready) {
    vl53l5cx_get_ranging_data(&_config, &_results);
    // Extract the middle row since this is a 3D rangefinder, but PX4 obstacle dist array is 2D
    for(int index = 39; index > 31; --index) {

      distance_indx = 39 - index;

      uint16_t distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
      obs.distances[distance_indx] = distance_val;
      _signal_qual[distance_indx] = _results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*index];
    
      switch(_signal_qual[distance_indx]) {
        // Range valid
	case 5:
	    _confidence_score = 100;
            RCLCPP_INFO(this->get_logger(), "Target detected");
	    break;

	// No targets detected, range valid
	case 255:
	    _confidence_score = 100;
	    break;
	
	// No previous target detected, range valid
	case 10:
	    _confidence_score = 100;
	    break;
	// Wrap around not performed (first range)
	case 6:
	    _confidence_score = 50;
            RCLCPP_WARN(this->get_logger(), "WARNING: Range quality poor");
	    break;
	// Range valid with large pulse
	case 9:
	    _confidence_score = 50;
            RCLCPP_WARN(this->get_logger(), "WARNING: Range quality poor");
	    break;
	// Range quality poor
	default:
	    _confidence_score = 0;
            RCLCPP_WARN(this->get_logger(), "WARNING: Range quality invalid: %hhu", _signal_qual[distance_indx]);
	    break;
      } // End switch
    } // End for

    // Check if we are subscribed to PX4 timesync
    if (_synced) {
        obs.timestamp = _synced_ts;
    } else {
        // Fall back on internal timer based on when the sensor node was started
    	obs.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(sensor_time_end - _sensor_time_start).count();
  
        RCLCPP_WARN(this->get_logger(), "WARNING: Not time synced with FMU");
    }
    
    // In 8x8 ranging mode the FoV is 60 degrees
    obs.angle_offset = -30.0f;
    obs.increment = 8.0f;
   
    // Coordinate frame is body FRD
    obs.frame = 12; // MAV_FRAME_BODY_FRD

    // Sensor limits in centimeters
    obs.min_distance = 2;
    obs.max_distance = 400;
    _obs_distance_msg->publish(obs);

    // reset time sync in the event we lose timesync from px4
    _synced = false;

  } // End if

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetsonAvoidance>());
  rclcpp::shutdown();
  return 0;
}
