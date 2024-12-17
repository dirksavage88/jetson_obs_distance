#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "px4_msgs/msg/obstacle_distance.hpp"


using std::placeholders::_1;

class PX4ObstacleDist: public rclcpp::Node {
  public:
  explicit PX4ObstacleDist () : Node("px4_obs_distance"){

    rmw_qos_profile_t sens_qos_profile = rmw_qos_profile_sensor_data;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(sens_qos_profile.history, 5), sens_qos_profile); 

    // _right_laser_subscription = this->create_subscription("/right/obstacle_distance", qos, std::bind(&PX4ObstacleDist::DistanceCB, this, _1));
    // _rear_laser_subscription = this->create_subscription("/rear/obstacle_distance", qos, std::bind(&PX4ObstacleDist::DistanceCB, this, _1));
    // _left_laser_subscription = this->create_subscription("/left/obstacle_distance", qos, std::bind(&PX4ObstacleDist::DistanceCB, this, _1));
    
    _px4_distance_pub = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
    _front_laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>("/front/obstacle_distance", qos, std::bind(&PX4ObstacleDist::FrontDistanceCB, this, _1));
  }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _front_laser_subscription;
    // rclcpp::Subscription::SharedPtr _right_laser_subscription;
    // rclcpp::Subscription::SharedPtr _rear_laser_subscription;
    // rclcpp::Subscription::SharedPtr _left_laser_subscription;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr _px4_distance_pub;
    const uint8_t _OBS_DISTANCE_MAX{72};
    const uint16_t _OUT_OF_RANGE{401};
    const uint8_t _ZONES{8};
    const uint8_t _M_TO_CM{100};
    uint16_t _front_map[8];
    uint16_t _right_map[8];
    uint16_t _rear_map[8];
    uint16_t _left_map[8];
    void FrontDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr);
};

void PX4ObstacleDist::FrontDistanceCB(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {

  px4_msgs::msg::ObstacleDistance ob_distance;

  ob_distance.angle_offset = -20.0f;
  ob_distance.increment = 5;
  ob_distance.frame = 12;
  ob_distance.min_distance = 5;
  ob_distance.max_distance = 400;
  // Put laserscan data into local array
  for (uint8_t i = 0; i < _OBS_DISTANCE_MAX; ++i) {
    // _front_map[i] = _M_TO_CM * _msg.ranges[i];
    if(i < 8) {

      ob_distance.distances[i] = _M_TO_CM * _msg->ranges[i];
    }
    else {
      ob_distance.distances[i] = _OUT_OF_RANGE;
    }

  }
  ob_distance.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  _px4_distance_pub->publish(ob_distance);

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4ObstacleDist>()); 
  rclcpp::shutdown();
  return 0;
}
