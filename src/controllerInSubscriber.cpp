#include <quad_sim/controllerInSubscriber.hpp>

controllerInSubscriber::controllerInSubscriber() : Node("controllerInSubscriber")
{
    armState_Sub = create_subscription<quad_sim_interfaces::msg::ArmState>("armed", rclcpp::SensorDataQoS(), std::bind(&controllerInSubscriber::armState_Scb, this, std::placeholders::_1));
}

void controllerInSubscriber::armState_Scb(quad_sim_interfaces::msg::ArmState msg)
{
    armState_lts = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec;
    armed = msg.armed;
}

bool controllerInSubscriber::isArmed()
{
    return armed;
}

bool controllerInSubscriber::isInSync(const int64_t &simTime_ns)
{
    return (armState_lts == simTime_ns);
}