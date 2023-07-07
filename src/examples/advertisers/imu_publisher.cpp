#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp> 

using namespace std::chrono_literals;

/**
 * @brief Sensor Combined uORB topic data callback
 */
class imuPublisher : public rclcpp::Node
{
public:
    
	explicit imuPublisher() : Node("imu_publisher")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);		
		
		publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("sensor_msgs/msg/Imu", 10);
		
		subscription_imu_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
		[this](const px4_msgs::msg::SensorCombined::UniquePtr raw_data_) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << raw_data_->timestamp    << std::endl;
			std::cout << "gyro_rad[0]: " << raw_data_->gyro_rad[0]  << std::endl;
			std::cout << "gyro_rad[1]: " << raw_data_->gyro_rad[1]  << std::endl;
			std::cout << "gyro_rad[2]: " << raw_data_->gyro_rad[2]  << std::endl;
			std::cout << "gyro_integral_dt: " << raw_data_->gyro_integral_dt << std::endl;
			std::cout << "accelerometer_timestamp_relative: " << raw_data_->accelerometer_timestamp_relative << std::endl;
			std::cout << "accelerometer_m_s2[0]: " << raw_data_->accelerometer_m_s2[0] << std::endl;
			std::cout << "accelerometer_m_s2[1]: " << raw_data_->accelerometer_m_s2[1] << std::endl;
			std::cout << "accelerometer_m_s2[2]: " << raw_data_->accelerometer_m_s2[2] << std::endl;
			std::cout << "accelerometer_integral_dt: " << raw_data_->accelerometer_integral_dt << std::endl;
			
		});
		
		subscription_q_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
		[this](const px4_msgs::msg::VehicleAttitude::UniquePtr raw_data_q_) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE ATTITUDE DATA"   << std::endl;
			std::cout << "============================="    << std::endl;
			std::cout << "ts: "                             << raw_data_q_->timestamp    << std::endl;
			std::cout << "ts sample: "                      << raw_data_q_->timestamp_sample  << std::endl;
			std::cout << "q[0]: "                              << raw_data_q_->q[0]  << std::endl;
			std::cout << "q[1]: "                              << raw_data_q_->q[0]  << std::endl;
			std::cout << "q[2]: "                              << raw_data_q_->q[0]  << std::endl;
			std::cout << "q[3]: "                              << raw_data_q_->q[0]  << std::endl;
			std::cout << "delta_q_reset[0]: "                  << raw_data_q_->delta_q_reset[0]  << std::endl;
			std::cout << "delta_q_reset[1]: "                  << raw_data_q_->delta_q_reset[1]  << std::endl;
			std::cout << "delta_q_reset[2]: "                  << raw_data_q_->delta_q_reset[2]  << std::endl;
			std::cout << "delta_q_reset[3]: "                  << raw_data_q_->delta_q_reset[3]  << std::endl;
			std::cout << "quat_reset_counter: "             << raw_data_q_->quat_reset_counter << std::endl;
			
		});
		
		//~ auto timer_callback = [this]()->void {
			//~ auto data_raw_pub_ = sensor_msgs::msg::Imu();

            //~ // get raw data from px4 topic
            //~ data_raw_pub_.header.seq = 0;
            //~ data_raw_pub_.header.stamp = current_time;
            //~ //data_raw_pub_.header.frame_id = "body_frame";
            //~ data_raw_pub_.orientation.x = raw_data_q_->q[0];
            //~ data_raw_pub_.orientation.y = raw_data_q_->q[1];
            //~ data_raw_pub_.orientation.z = raw_data_q_->q[2];
            //~ data_raw_pub_.orientation.w = raw_data_q_->q[3];
            //~ data_raw_pub_.angular_velocity.x = raw_data_->gyro_rad[0];
            //~ data_raw_pub_.angular_velocity.y = raw_data_->gyro_rad[1];
            //~ data_raw_pub_.angular_velocity.z = raw_data_->gyro_rad[2];
            //~ data_raw_pub_.linear_acceleration.x = raw_data_->accelerometer_m_s2[0];
            //~ data_raw_pub_.linear_acceleration.y = raw_data_->accelerometer_m_s2[1];
            //~ data_raw_pub_.linear_acceleration.z = raw_data_->accelerometer_m_s2[2];

			//~ std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			//~ std::cout << "PUBLISHED SENSOR COMBINED DATA"   << std::endl;
			//~ std::cout << "============================="    << std::endl;
			//~ std::cout << "seq "                             << data_raw_pub_.header.seq = 0;
			//~ std::cout << "ts: "                             << data_raw_pub_.header.stamp    << std::endl;
			//~ //std::cout << "frame_id "                        << data_raw_pub_.header.frame_id = "body_frame";
            //~ std::cout << "orientation.x "                   << data_raw_pub_.orientation.x = raw_data_q_->q[0];
            //~ std::cout << "orientation.y "                   << data_raw_pub_.orientation.y = raw_data_q_->q[1];
            //~ std::cout << "orientation.z "                   << data_raw_pub_.orientation.z = raw_data_q_->q[2];
            //~ std::cout << "orientation.w "                   << data_raw_pub_.orientation.w = raw_data_q_->q[3];
			//~ std::cout << "angular_velocity.x "              << data_raw_pub_.angular_velocity.x  << std::endl;
			//~ std::cout << "angular_velocity.y "              << data_raw_pub_.angular_velocity.y  << std::endl;
			//~ std::cout << "angular_velocity.z "              << data_raw_pub_.angular_velocity.z  << std::endl;
			//~ std::cout << "linear_acceleration.x "           << data_raw_pub_.linear_acceleration.x << std::endl;
			//~ std::cout << "linear_acceleration.y "           << data_raw_pub_.linear_acceleration.y << std::endl;
			//~ std::cout << "linear_acceleration.z "           << data_raw_pub_.linear_acceleration.z << std::endl;

            //~ // publish ros2 IMU msg
			//~ this->publisher_->publish(data_raw_pub_);
		//~ };

		//~ timer_ = this->create_wall_timer(500ms, timer_callback);
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_imu_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_q_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting imu publisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imuPublisher>());

	rclcpp::shutdown();
	return 0;
}


