#include <memory>
#include <random>
#include <cmath> // Include for sqrt function

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class RandGauTFPub : public rclcpp::Node
{
public:
    RandGauTFPub()
    : Node("rand_gau_tf_pub"), rd(), gen_(rd()), dist_(-0.1, 0.1)
    {
        RCLCPP_INFO(this->get_logger(), "Node initialized");

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/omniFPS/Robots/FloatingPlatform/PoseStamped", 10,
            std::bind(&RandGauTFPub::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/omniFPS/Robots/FloatingPlatform/PoseStampedNoised", 10);
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped &msg)
    {
        auto noisy_msg = msg;

        noisy_msg.pose.position.x += dist_(gen_);
        noisy_msg.pose.position.y += dist_(gen_);
        noisy_msg.pose.position.z += dist_(gen_);

        noisy_msg.pose.orientation.x += dist_(gen_);
        noisy_msg.pose.orientation.y += dist_(gen_);
        noisy_msg.pose.orientation.z += dist_(gen_);
        noisy_msg.pose.orientation.w += dist_(gen_);

        publisher_->publish(noisy_msg);
    }

    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::random_device rd;                       // Random device
    std::mt19937 gen_;                           // Mersenne Twister generator
    std::uniform_real_distribution<double> dist_; // Uniform distribution
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandGauTFPub>());
    rclcpp::shutdown();
    return 0;
}
