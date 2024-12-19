#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"
#include <chrono>
#include <cstdio>

using std::placeholders::_1;

class PX4ObstacleDist: public rclcpp::Node {
  public:
  explicit PX4ObstacleDist () : Node("px4_obs_distance"){

    // PX4 topics should be qos profile sensor data
    //rmw_qos_profile_t sens_qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    //auto qos = rclcpp::QoS(rclcpp::QoSInitialization(sens_qos_profile.history, 5), sens_qos_profile); 
    
    // Publishers
    _px4_distance_pub = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
  
    // Subscriptions
    _front_laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/front/obstacle_distance", qos, std::bind(&PX4ObstacleDist::FrontDistanceCB, this, _1));
    _right_laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/right/obstacle_distance", qos, std::bind(&PX4ObstacleDist::RightDistanceCB, this, _1));
    _rear_laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/rear/obstacle_distance", qos, std::bind(&PX4ObstacleDist::RearDistanceCB, this, _1));
    _left_laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/left/obstacle_distance", qos, std::bind(&PX4ObstacleDist::LeftDistanceCB, this, _1));
    timer_cb_group_ = nullptr;
    // Publish to the px4 obstacle map only at ~3hz, since sensor data is 5hz
    timer_ptr_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&PX4ObstacleDist::SendDistance, this));
  }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _front_laser_subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _right_laser_subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _rear_laser_subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _left_laser_subscription;
    
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _px4_distance_pub;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    const uint8_t _OBS_DISTANCE_MAX{72};
    const uint16_t _OUT_OF_RANGE{401};
    const uint8_t _ZONES{8};
    const uint8_t _M_TO_CM{100};
    uint16_t _front_map[8]{401, 401, 401, 401, 401, 401, 401, 401};
    uint16_t _right_map[8]{401, 401, 401, 401, 401, 401, 401, 401};
    uint16_t _rear_map[8]{401, 401, 401, 401, 401, 401, 401, 401};
    uint16_t _left_map[8]{401, 401, 401, 401, 401, 401, 401, 401};
    void FrontDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr);
    void RightDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr);
    void RearDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr);
    void LeftDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr);
    void SendDistance();
};
void PX4ObstacleDist::SendDistance() {

  px4_msgs::msg::ObstacleDistance ob_distance;
  
  // Each sensor has ~40 degree horizontal FoV
  ob_distance.angle_offset = -20.0f;
  ob_distance.increment = 5;
  ob_distance.frame = 12;
  ob_distance.min_distance = 5;
  ob_distance.max_distance = 400;
  // Put laserscan data into local array
  for (uint8_t i = 0; i < _OBS_DISTANCE_MAX; ++i) {
    // _front_map[i] = _M_TO_CM * _msg.ranges[i];
    if(i < 8) {

      ob_distance.distances[i] = _front_map[i];
    }

    else if(i > 17 && i < 26) {

      ob_distance.distances[i] = _right_map[i - 18];

    }

    else if (i > 35 && i < 44) {

      ob_distance.distances[i] = _rear_map[i - 36];

    }

    else if (i > 53 && i < 62) {

      ob_distance.distances[i] = _left_map[i - 54];

    }
    else {
      ob_distance.distances[i] = _OUT_OF_RANGE;
    }

  }
  ob_distance.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  _px4_distance_pub->publish(ob_distance);


}
void PX4ObstacleDist::FrontDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
  // Put laserscan data into local array
  for (uint8_t i = 0; i < _ZONES; ++i) {
    _front_map[i] = _M_TO_CM * _msg->ranges[i];

  }
}

void PX4ObstacleDist::RightDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

  // Put laserscan data into local array
  for (uint8_t i = 0; i < _ZONES; ++i) {
    _right_map[i] = _M_TO_CM * _msg->ranges[i];
  }
}
void PX4ObstacleDist::RearDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

  // Put laserscan data into local array
  for (uint8_t i = 0; i < _ZONES; ++i) {
    _rear_map[i] = _M_TO_CM * _msg->ranges[i];
  }
}
void PX4ObstacleDist::LeftDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

  // Put laserscan data into local array
  for (uint8_t i = 0; i < _ZONES; ++i) {
    _left_map[i] = _M_TO_CM * _msg->ranges[i];
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4ObstacleDist>()); 
  rclcpp::shutdown();
  return 0;
}
