#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;

/**
 * @brief Sensor Combined uORB topic data callback
 */
class odomPublisher : public rclcpp::Node
{
public:
    
	explicit odomPublisher() : Node("odom_publisher")
	{
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);		
		
		publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("nav_msgs/Odometry", 10);
		
		subscription_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
		[this](const px4_msgs::msg::VehicleOdometry::UniquePtr raw_data_) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED ODOM DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "                 << raw_data_->timestamp    << std::endl;
			std::cout << "position.x: "         << raw_data_->position[0]          << std::endl;
			std::cout << "position.y: "         << raw_data_->position[1]          << std::endl;
			std::cout << "position.z: "         << raw_data_->position[2]          << std::endl;	
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_odom_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting odom publisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odomPublisher>());

	rclcpp::shutdown();
	return 0;
}


