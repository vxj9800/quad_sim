// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#ifndef __SIMULATION_TIME_PUBLISHER_HEADER__
#define __SIMULATION_TIME_PUBLISHER_HEADER__

class simTimePublisher : public rclcpp::Node
{
public:
    // Contructor
    simTimePublisher();

    // Publisher functions
    void simTime_PFn(int64_t simTime_ns);

private:
    // Variables for subscribers
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr simTime_Pub;
};

#endif // __SIMULATION_TIME_PUBLISHER_HEADER__