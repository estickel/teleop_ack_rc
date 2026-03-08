#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "teleop_ack_rc/teleop_ack_rc_node.hpp"

int main(int argc, char * argv[])
{
    /* Initialize ROS 2 */
    rclcpp::init(argc, argv);

    /* Create the node and spin it */
    auto node = std::make_shared<teleop_ack_rc::TeleopAckRcNode>(rclcpp::NodeOptions());
    
    RCLCPP_INFO(node->get_logger(), "RC Teleop Node started on /dev/ttyACM1");
    
    rclcpp::spin(node);

    /* Shutdown cleanly */
    rclcpp::shutdown();
    return 0;
}