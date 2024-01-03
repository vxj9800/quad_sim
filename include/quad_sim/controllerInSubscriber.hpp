// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <quad_sim_interfaces/msg/arm_state.hpp>
#include <quad_sim_interfaces/msg/quad_esc.hpp>

#ifndef __CONTROLLER_IN_SUBSCRIBER_HEADER__
#define __CONTROLLER_IN_SUBSCRIBER_HEADER__

class controllerInSubscriber : public rclcpp::Node
{
public:
    // Contructor
    controllerInSubscriber();

    // Provide arming state
    bool getArmState();

    // Provide motor voltages
    void getMotVolts(const double &battVolts, std::vector<double> &motVolts);

private:
    // Variables for subscribers
    rclcpp::Subscription<quad_sim_interfaces::msg::ArmState>::SharedPtr armState_Sub;
    rclcpp::Subscription<quad_sim_interfaces::msg::QuadESC>::SharedPtr motEsc_Sub;

    // Subscriber Callback functions
    void armState_SCb(quad_sim_interfaces::msg::ArmState msg);
    void motEsc_SCb(quad_sim_interfaces::msg::QuadESC msg);

    // Variables to store received data
    bool armed = false;
    double escB = 0, escC = 0, escD = 0, escE = 0;
};

#endif // __CONTROLLER_IN_SUBSCRIBER_HEADER__