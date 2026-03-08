#ifndef TELEOP_ACK_RC_NODE_HPP_
#define TELEOP_ACK_RC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <libserial/SerialPort.h>

namespace teleop_ack_rc {

/* RC Packet the microcontroller sends. */
struct __attribute__((packed)) RC_PACKET {
    uint8_t header;     /* 0xAA */
    uint16_t steer;     /* CH0 (8) */
    uint16_t throttle;  /* CH1 (9) */
    uint16_t teleop;    /* CH2 (10) */
};

class TeleopAckRcNode : public rclcpp::Node {
public:
    explicit TeleopAckRcNode(const rclcpp::NodeOptions & options);
    virtual ~TeleopAckRcNode();

private:
    void timer_callback();
    double map_value(uint16_t x, uint16_t in_min, uint16_t in_max, double out_min, double out_max);

    /* Serial */
    LibSerial::SerialPort remote_control_port_;
    std::string port_name_;
    
    /* Publishers */
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr ack_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    
    /* Timer for polling serial */
    rclcpp::TimerBase::SharedPtr timer_;

    /* Parameters */
    double max_speed_;
    double max_steering_angle_;
};

} /* namespace teleop_ack_rc */

#endif  /* TELEOP_ACK_RC_NODE_HPP_ */