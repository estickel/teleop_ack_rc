#include "teleop_ack_rc/teleop_ack_rc_node.hpp"
#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace teleop_ack_rc
{

TeleopAckRcNode::TeleopAckRcNode(const rclcpp::NodeOptions & options)
    : Node("teleop_ack_rc_node", options)
{
    /* 1. Initialize parameters with measured hardware defaults */
    this->declare_parameter("port", "/dev/ttyACM1");
    this->declare_parameter("max_speed", 6.7056);           
    this->declare_parameter("max_steering_angle", 0.2733);  

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
        RCLCPP_INFO(this->get_logger(), "Connected to Phoenix Microcode on %s", port_name_.c_str());
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
    while (remote_control_port_.IsDataAvailable()) {
        uint8_t header;
        remote_control_port_.ReadByte(header);

        if (header == 0xAA) {
            RC_PACKET p;
            p.header = header;
            std::vector<uint8_t> buffer;

            try {
                /* Read the 6-byte payload (steer, throttle, teleop uint16_ts) */
                remote_control_port_.Read(buffer, 6, 20);
                if (buffer.size() == 6) {
                    std::memcpy(&p.steer, buffer.data(), 6);

                    /* --- Normalization Lambda with Reliability Guardbands --- */
                    auto apply_guardbands = [](uint16_t raw, uint16_t min_r, uint16_t max_r) {
                        const double CENTER_LOW = 1490.0;
                        const double CENTER_HIGH = 1510.0;
                        double val = static_cast<double>(raw);

                        /* 1. Handle Deadzone (Maps to 0.0) */
                        if (val >= CENTER_LOW && val <= CENTER_HIGH) return 0.0;
                        
                        /* 2. Handle Positive Range (Maps to 0.0 -> 1.0) */
                        if (val > CENTER_HIGH) {
                            double out = (val - CENTER_HIGH) / (static_cast<double>(max_r) - CENTER_HIGH);
                            return std::min(1.0, out);
                        } 
                        /* 3. Handle Negative Range (Maps to -1.0 -> 0.0) */
                        else {
                            double out = (val - CENTER_LOW) / (CENTER_LOW - static_cast<double>(min_r));
                            return std::max(-1.0, out);
                        }
                    };

                    /* --- Apply Specific Measured Hardware Limits --- */
                    double norm_steer = apply_guardbands(p.steer, 1044, 1996);
                    double norm_throttle = apply_guardbands(p.throttle, 1016, 1920);

                    /* 1. Publish Ackermann Drive */
                    auto ack_msg = ackermann_msgs::msg::AckermannDrive();
                    /* Standard convention: Positive steering angle = Left turn */
                    ack_msg.steering_angle = norm_steer * -max_steering_angle_;
                    ack_msg.speed = norm_throttle * max_speed_;
                    ack_pub_->publish(ack_msg);

                    /* 2. Publish Joy message (The Spoof) */
                    auto joy_msg = sensor_msgs::msg::Joy();
                    joy_msg.header.stamp = this->now();
                    joy_msg.axes = {(float)norm_steer, (float)norm_throttle};

                    /* Direct Button Mapping for DriveModeSwitch compatibility.
                       We assign 10 buttons (initialized to 0) to ensure index 8 exists.
                    */
                    joy_msg.buttons.assign(10, 0); 
                    
                    /* Switch State -> Button State:
                       If Switch is High (>1500), Button 8 is Pressed (1).
                       If Switch is Low (<1500), Button 8 is Released (0).
                    */
                    joy_msg.buttons[8] = (p.teleop > 1500) ? 1 : 0; 

                    joy_pub_->publish(joy_msg);
                }
            } catch (...) {
                /* Ignore packet sync errors */
            }
        }
    }
}

} /* namespace teleop_ack_rc */

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(teleop_ack_rc::TeleopAckRcNode)