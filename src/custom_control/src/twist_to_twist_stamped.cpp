#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>

std::string SUBSCRIPTION_TOPIC = "/cmd_vel";
int QOS_HISTORY_DEPTH = 10;
std::string PUBLISHER_TOPIC = "/diff_drive_controller/cmd_vel";

class TwistStampedPublisher : public rclcpp::Node {
    public:
        TwistStampedPublisher() : rclcpp::Node("twist_to_twist_stamped") {
            // Parameterize subscription and publisher topics for flexibility 
            this->declare_parameter("subscription_topic", SUBSCRIPTION_TOPIC);
            this->declare_parameter("publisher_topic", PUBLISHER_TOPIC);

            this->sub = this->create_subscription<geometry_msgs::msg::Twist>(
                this->get_parameter("subscription_topic").as_string(),
                QOS_HISTORY_DEPTH,
                std::bind(
                    &TwistStampedPublisher::sub_callback,
                    this,
                    std::placeholders::_1)
                );
            this->pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                this->get_parameter("publisher_topic").as_string(),
                QOS_HISTORY_DEPTH);

            this->frame_id = 1;
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub;
        int frame_id;

        void sub_callback(const geometry_msgs::msg::Twist& twist) {
            std_msgs::msg::Header header;
            header.frame_id = std::to_string(this->frame_id);
            header.stamp = this->get_clock()->now();  // Get current time from clock used by node
            geometry_msgs::msg::TwistStamped twist_stamped;
            twist_stamped.header = header;
            twist_stamped.twist = twist;
            this->pub->publish(twist_stamped);

            this->frame_id++;
        }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistStampedPublisher>());
    rclcpp::shutdown();
}
