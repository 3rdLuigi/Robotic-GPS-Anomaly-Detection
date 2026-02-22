#include <chrono>
#include <memory>
#include <random>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class SensorNoiseNode : public rclcpp::Node
{
public:
  SensorNoiseNode() : Node("sensor_noise_node")
  {
    // 1. Setup Random Number Generators for Math Modeling
    std::random_device rd;
    gen_ = std::mt19937(rd());
    jitter_dist_ = std::normal_distribution<>(0.0, 0.5); 
    outlier_chance_dist_ = std::uniform_real_distribution<>(0.0, 1.0);
    outlier_spike_dist_ = std::normal_distribution<>(0.0, 15.0); 

    // 2. Initialize and open the CSV file
    csv_file_.open("src/gps_anomaly_sim/matlab/turtle_trajectory.csv");
    if (csv_file_.is_open()) {
      // Write the CSV headers
      csv_file_ << "time,x_true,y_true,x_noisy,y_noisy\n";
      RCLCPP_INFO(this->get_logger(), "Logging data directly to turtle_trajectory.csv");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file.");
    }

    // 3. Setup ROS Pub/Sub
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&SensorNoiseNode::pose_callback, this, _1));
      
    publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/noisy_pose", 10);
  }

  // Destructor to safely close the file when you kill the node
  ~SensorNoiseNode()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV file closed and saved successfully.");
    }
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    auto noisy_msg = turtlesim::msg::Pose();
    noisy_msg = *msg; 

    // Inject constant Gaussian jitter
    noisy_msg.x += jitter_dist_(gen_);
    noisy_msg.y += jitter_dist_(gen_);

    // Inject severe outliers (2% chance)
    if (outlier_chance_dist_(gen_) < 0.02) {
      noisy_msg.x += outlier_spike_dist_(gen_);
      noisy_msg.y += outlier_spike_dist_(gen_);
      RCLCPP_WARN(this->get_logger(), "Outlier Injected!");
    }

    // Publish the anomalous data to ROS (optional now, but good practice)
    publisher_->publish(noisy_msg);

    // 4. WRITE TO CSV: Save current timestamp, perfect data, and noisy data
    if (csv_file_.is_open()) {
      csv_file_ << this->now().seconds() << ","
                << msg->x << ","
                << msg->y << ","
                << noisy_msg.x << ","
                << noisy_msg.y << "\n";
    }
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
  std::mt19937 gen_;
  std::normal_distribution<> jitter_dist_;
  std::uniform_real_distribution<> outlier_chance_dist_;
  std::normal_distribution<> outlier_spike_dist_;
  std::ofstream csv_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNoiseNode>());
  rclcpp::shutdown();
  return 0;
}