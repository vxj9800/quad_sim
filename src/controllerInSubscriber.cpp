#include <quad_sim/controllerInSubscriber.hpp>

controllerInSubscriber::controllerInSubscriber() : Node("controllerInSubscriber")
{
    armState_Sub = create_subscription<quad_sim_interfaces::msg::ArmState>("armed", rclcpp::SensorDataQoS(), std::bind(&controllerInSubscriber::armState_SCb, this, std::placeholders::_1));
    motEsc_Sub = create_subscription<quad_sim_interfaces::msg::QuadESC>("motEsc", rclcpp::SensorDataQoS(), std::bind(&controllerInSubscriber::motEsc_SCb, this, std::placeholders::_1));
}

void controllerInSubscriber::armState_SCb(quad_sim_interfaces::msg::ArmState msg)
{
    armed = msg.armed;
}

void controllerInSubscriber::motEsc_SCb(quad_sim_interfaces::msg::QuadESC msg)
{
    escB = msg.mot_b;
    escC = msg.mot_c;
    escD = msg.mot_d;
    escE = msg.mot_e;
}

void controllerInSubscriber::getMotVolts(const double &battVolts, std::vector<double> &motVolts)
{
    motVolts[0] = battVolts * escB;
    motVolts[1] = battVolts * escC;
    motVolts[2] = battVolts * escD;
    motVolts[3] = battVolts * escE;
}

bool controllerInSubscriber::getArmState()
{
    return armed;
}