#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp> 

using namespace std::chrono_literals;

/**
 * @brief Sensor Combined uORB topic data callback
 */
class gpsPublisher : public rclcpp::Node
{
public:
    
	explicit gpsPublisher() : Node("gps_publisher")
	{
		
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);		
		
		publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("sensor_msgs/NavSatFix", 10);
		
		subscription_gps_ = this->create_subscription<px4_msgs::msg::SensorGps>("/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr raw_data_) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED GPS DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << raw_data_->timestamp    << std::endl;
			std::cout << "lat: "         << raw_data_->lat          << std::endl;
			std::cout << "lon: "         << raw_data_->lon          << std::endl;
			std::cout << "alt: "         << raw_data_->alt          << std::endl;	
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_gps_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting gps publisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gpsPublisher>());

	rclcpp::shutdown();
	return 0;
}


