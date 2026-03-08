#include "teleop_ack_rc/teleop_ack_rc_node.hpp"
#include <vector>
#include <cstring>
#include <cmath>

namespace teleop_ack_rc
{

TeleopAckRcNode::TeleopAckRcNode(const rclcpp::NodeOptions & options)
    : Node("teleop_ack_rc_node", options)
{
    /* 1. Initialize parameters with robust defaults */
    this->declare_parameter("port", "/dev/ttyACM1");
    this->declare_parameter("max_speed", 6.7056);
    this->declare_parameter("max_steering_angle", 0.2733);
    this->declare_parameter("deadzone", 15); // PWM microseconds of jitter to ignore

    port_name_ = this->get_parameter("port").as_string();
    max_speed_ = this->get_parameter("max_speed").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    ack_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("/ack_vel", 10);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    /* 2. Configure Serial Port */
    try {
        remote_control_port_.Open(port_name_);
        remote_control_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        remote_control_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        remote_control_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        remote_control_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        remote_control_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        RCLCPP_INFO(this->get_logger(), "Connected to Arduino on %s", port_name_.c_str());
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Serial Error: %s", e.what());
    }

    /* 3. Set timer to poll at 50Hz (20ms) */
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TeleopAckRcNode::timer_callback, this));
}

TeleopAckRcNode::~TeleopAckRcNode()
{
    if (remote_control_port_.IsOpen()) {
        remote_control_port_.Close();
    }
}

void TeleopAckRcNode::timer_callback()
{
    const double RC_NEUTRAL = 1500.0;
    const double RC_RANGE = 500.0;
    int deadzone_limit = this->get_parameter("deadzone").as_int();

    while (remote_control_port_.IsDataAvailable()) {
        uint8_t header;
        remote_control_port_.ReadByte(header);

        if (header == 0xAA) {
            RC_PACKET p;
            p.header = header;
            std::vector<uint8_t> buffer;

            try {
                remote_control_port_.Read(buffer, 6, 10);
                if (buffer.size() == 6) {
                    std::memcpy(&p.steer, buffer.data(), 6);

                    /* Helper Lambda: Map Raw PWM to Normalized Joy Axis [-1.0, 1.0] with Deadzone */
                    auto get_normalized_axis = [&](uint16_t raw) {
                        double offset = static_cast<double>(raw) - RC_NEUTRAL;
                        if (std::abs(offset) < deadzone_limit) {
                            return 0.0;
                        }
                        return offset / RC_RANGE;
                    };

                    double norm_steer = get_normalized_axis(p.steer);
                    double norm_throttle = get_normalized_axis(p.throttle);

                    /* 1. Map and Publish Ackermann Drive */
                    auto ack_msg = ackermann_msgs::msg::AckermannDrive();
                    /* Steering: Inverted mapping often needed (Right stick = Negative Angle) */
                    ack_msg.steering_angle = norm_steer * -max_steering_angle_;
                    ack_msg.speed = norm_throttle * max_speed_;
                    ack_pub_->publish(ack_msg);

                    /* 2. Publish Joy message for drive_mode_switch */
                    auto joy_msg = sensor_msgs::msg::Joy();
                    joy_msg.header.stamp = this->now();
                    joy_msg.axes.push_back(norm_steer);
                    joy_msg.axes.push_back(norm_throttle);
                    /* Button 0 high if teleop switch is active */
                    joy_msg.buttons.push_back((p.teleop > 1600) ? 1 : 0);
                    joy_pub_->publish(joy_msg);
                }
            } catch (...) {
                continue;
            }
        }
    }
}

} /* namespace teleop_ack_rc */

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(teleop_ack_rc::TeleopAckRcNode)