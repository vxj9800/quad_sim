#include <quad_sim/simTimePublisher.hpp>

simTimePublisher::simTimePublisher() : Node("simTimePublisher")
{
    // Initialize the Publishers
    simTime_Pub = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).reliable().durability_volatile());
}

void simTimePublisher::simTime_PFn(int64_t simTime_ns)
{
    // Create message variable
    rosgraph_msgs::msg::Clock msg;

    // Add data to the message variable
    msg.clock.sec = simTime_ns / 1000000000;
    msg.clock.nanosec = simTime_ns % 1000000000;

    // Publish the message
    simTime_Pub->publish(msg);
}