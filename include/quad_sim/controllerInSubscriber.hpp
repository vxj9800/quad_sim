// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <quad_sim_interfaces/msg/arm_state.hpp>

#ifndef __CONTROLLER_IN_SUBSCRIBER_HEADER__
#define __CONTROLLER_IN_SUBSCRIBER_HEADER__

class controllerInSubscriber : public rclcpp::Node
{
public:
    // Contructor
    controllerInSubscriber();

    // Provide arming state
    bool getArmState(int64_t &timeStamp);

private:
    // Variables for subscribers
    rclcpp::Subscription<quad_sim_interfaces::msg::ArmState>::SharedPtr armState_Sub;

    // Subscriber Callback functions
    void armState_Scb(quad_sim_interfaces::msg::ArmState msg);

    // Variables to store received data
    bool armed;

    // Keep track of last time-stamp in nanoseconds
    int64_t armState_lts = 0;
};

#endif // __CONTROLLER_IN_SUBSCRIBER_HEADER__