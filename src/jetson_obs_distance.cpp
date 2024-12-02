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

class JetsonAvoidance :  public rclcpp::Node 
{

  public:
  explicit JetsonAvoidance() : Node("jetson_obs_distance")
  {
    rmw_qos_profile_t sens_qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(sens_qos_profile.history, 5), sens_qos_profile);
    // Declare parameter for i2c bus on jetson (usually 0, 1, or 7)
    this->declare_parameter("i2c_bus", 1);
    this->declare_parameter("obs_index_offset", 0);
    
    // Default address is 0x52, or 82 in base 10
    this->declare_parameter("i2c_addr", 82);
    
    // Fill the publisher
    _obs_distance_msg = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);

    // Get the i2c bus
    _i2c_bus = this->get_parameter("i2c_bus").as_int();
    
    // If we are running multiple sensors or a sensor not facing forward, get the obstacle distance offset
    _obs_array_offset = this->get_parameter("obs_index_offset").as_int();

    _sensor_address = this->get_parameter("i2c_addr").as_int();
    // Configure the VL53L5CX
    SensorInit();

    // only create wall timer if sensor init was successful
     if(_return_status == 0) {
     
    	_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JetsonAvoidance::DistanceReadCB, this));
     }

  }

  private:

    void SensorInit(void);
    void DistanceReadCB(void);
    
    // Create publisher
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _obs_distance_msg;
    
    // VL53L5CX configuration & results struct
    VL53L5CX_Configuration _config;
    VL53L5CX_ResultsData _results;
    // Create Timer
    rclcpp::TimerBase::SharedPtr _timer;
    
    // Constants
    const uint16_t _UINT16_MAX{65535};
    const uint8_t _OBS_DISTANCE_MAX{72};
    
    // Member variables
    uint8_t _return_status{0};
    uint8_t _is_active{0};
    uint8_t _data_ready{0};
    uint8_t _init_retries{5};
    float _confidence_score{100};
    uint8_t _signal_qual[8];
    uint8_t _array_vertical_slice{31};
    uint8_t _zone_elements{8};
    int _sensor_address;
    int _i2c_bus{1};
    int _obs_array_offset{0};
};

void JetsonAvoidance::SensorInit()
{
  SensorState sensor_status = SensorState::Uninitialized;

  while(sensor_status != SensorState::InitComplete) {

    switch(sensor_status) {

      case SensorState::Uninitialized:
        _return_status = vl53l5cx_comms_init(&_config.platform, static_cast<char>(_i2c_bus), static_cast<uint16_t>(_sensor_address));

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
	// Set the ranging mode to continuous (VCSEL always on)
	_return_status |= vl53l5cx_set_ranging_mode(&_config, VL53L5CX_RANGING_MODE_CONTINUOUS);
	// Integration ignored if in continuous
        _return_status |= vl53l5cx_set_ranging_frequency_hz(&_config, 2);
        // Sharpener to delineate targets
	_return_status |= vl53l5cx_set_sharpener_percent(&_config, 2);
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
          sensor_status = SensorState::InitComplete;
          RCLCPP_INFO(this->get_logger(), "VL53L5CX Init Complete");
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

    }// End switch

  }// End while

}

void JetsonAvoidance::DistanceReadCB() {
  uint16_t distance_val;
  uint8_t distance_indx = 0;
  px4_msgs::msg::ObstacleDistance obs;

  vl53l5cx_check_data_ready(&_config, &_data_ready);
  
  // Set the distances outside of the sensor field of view as "unknown/not used"
  for (uint8_t i = _zone_elements; i < _OBS_DISTANCE_MAX; ++i){
    obs.distances[i] = _UINT16_MAX;

  }
  // Collect ranging results array from the VL53L5CX

  if(_data_ready) {
    vl53l5cx_get_ranging_data(&_config, &_results);
    // Extract the middle row since this is a 3D rangefinder, but PX4 obstacle dist array is 2D
    for(int index = _array_vertical_slice; index > (_array_vertical_slice - _zone_elements); --index) {
      /* Slicing the 8x8 array in half, we roughly get 23-39 as the middle two rows. 
        Start at lower slice since it is better to get a slightly upward distance upward 
	above the frame's horizon if we are pitched forward (and the SPAD array zones are
       	flipped across X and Y axis, so lower SPAD slices detect objects physically higher)
      */
      distance_indx = _array_vertical_slice - index;

      // Determine the signal quality first, then read the ranging result (or set to UINT16_MAX)
      _signal_qual[distance_indx] = _results.target_status[VL53L5CX_NB_TARGET_PER_ZONE*index];
    
      switch(_signal_qual[distance_indx]) {

	case 0:
	   _confidence_score = 25;
           RCLCPP_WARN(this->get_logger(), "Ranging not updated");
           distance_val = _UINT16_MAX;
	   break;
       case 4:
	   _confidence_score = 25;
           RCLCPP_INFO(this->get_logger(), "Target consistency failed");
           distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
	   break;
      
        // Range valid. 
        case 5:
	   _confidence_score = 100;
           RCLCPP_INFO(this->get_logger(), "Target detected");
           distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
	   break;
          
	case 6:
	   _confidence_score = 50;
           RCLCPP_INFO(this->get_logger(), "Wrap around not performed");
           distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
	   break;
	case 9:
	   _confidence_score = 50;
           RCLCPP_INFO(this->get_logger(), "Range valid with large pulse");
           distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
	   break;
	
	// No previous target detected+range valid
	case 10:
	   _confidence_score = 0;
           RCLCPP_INFO(this->get_logger(), "Range valid, no target detected previously");
	   distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1);
	   break;

	case 12:
	   _confidence_score = 50;
           RCLCPP_INFO(this->get_logger(), "Target blurred");
           distance_val = round(_results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE*index] * 0.1); 
	   break;
	// Ignore since if the target is removed from fov, the buffer will not be zeroed out = stale data
	case 255:
	   _confidence_score = 0;
           RCLCPP_WARN(this->get_logger(), "No target detected");
           distance_val = _UINT16_MAX;
	   break;
	// Range quality invalid
	default:
	   _confidence_score = 0;
           RCLCPP_WARN(this->get_logger(), "WARNING: Range quality invalid: %hhu", _signal_qual[distance_indx]);
           distance_val = _UINT16_MAX;
	   break;
      } // End switch

      // Invalidate (set to unknown) if distance value returns 0 m range
      if (distance_val == 0) {
      	distance_val = _UINT16_MAX;
      }
      
      // Publish the results to the array
      obs.distances[distance_indx + _obs_array_offset] = distance_val;
      //RCLCPP_INFO(this->get_logger(), "Range: %hu", distance_val);
    } // End for

    	obs.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  
    // In 8x8 ranging mode the FoV is ~63 degrees, each of the 8 elements increment is 10 deg to divide by 360 evenly
    obs.angle_offset = -40.0f;
    obs.increment = 10;
   
    // Coordinate frame is body FRD
    obs.frame = 12; // MAV_FRAME_BODY_FRD

    // Sensor limits in centimeters
    obs.min_distance = 5;
    obs.max_distance = 400;
    _obs_distance_msg->publish(obs);


  } // End if

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetsonAvoidance>());
  rclcpp::shutdown();
  return 0;
}
