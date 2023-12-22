#include <quad_sim/animStatePublisher.h>

animStatePublisher::animStatePublisher(double in_dt) : Node("animStatePublisher"), dt(in_dt)
{
    // Initialize the Publishers
    bodyPose_Pub = this->create_publisher<geometry_msgs::msg::Pose>("quadPose", rclcpp::SensorDataQoS());
    motB_Pub = this->create_publisher<std_msgs::msg::Float64>("motB", rclcpp::SensorDataQoS());
    motC_Pub = this->create_publisher<std_msgs::msg::Float64>("motC", rclcpp::SensorDataQoS());
    motD_Pub = this->create_publisher<std_msgs::msg::Float64>("motD", rclcpp::SensorDataQoS());
    motE_Pub = this->create_publisher<std_msgs::msg::Float64>("motE", rclcpp::SensorDataQoS());
    tick_Pub = this->create_publisher<builtin_interfaces::msg::Time>("tick", rclcpp::SensorDataQoS());
}

void animStatePublisher::bodyPose_PFun(std::vector<double> bodyPose)
{
    // Create message variable
    geometry_msgs::msg::Pose msg;

    // Add position data to the message variable
    msg.position.x = bodyPose[0];
    msg.position.y = bodyPose[1];
    msg.position.z = bodyPose[2];

    // Add orientation data to the message variable
    msg.orientation.w = bodyPose[3];
    msg.orientation.x = bodyPose[4];
    msg.orientation.y = bodyPose[5];
    msg.orientation.z = bodyPose[6];

    // Publish the message
    bodyPose_Pub->publish(msg);
}

void animStatePublisher::motB_PFun(double motAng)
{
    // Create message variable
    std_msgs::msg::Float64 msg;

    // Add data to the message variable
    msg.data = motAng;

    // Publish the message
    motB_Pub->publish(msg);
}

void animStatePublisher::motC_PFun(double motAng)
{
    // Create message variable
    std_msgs::msg::Float64 msg;

    // Add data to the message variable
    msg.data = motAng;

    // Publish the message
    motC_Pub->publish(msg);
}

void animStatePublisher::motD_PFun(double motAng)
{
    // Create message variable
    std_msgs::msg::Float64 msg;

    // Add data to the message variable
    msg.data = motAng;

    // Publish the message
    motD_Pub->publish(msg);
}

void animStatePublisher::motE_PFun(double motAng)
{
    // Create message variable
    std_msgs::msg::Float64 msg;

    // Add data to the message variable
    msg.data = motAng;

    // Publish the message
    motE_Pub->publish(msg);
}

void animStatePublisher::tick_PFun(double tick)
{
    // Create message variable
    builtin_interfaces::msg::Time msg;

    // Add second and nanosecond values to the message variable
    msg.sec = (int)tick;
    msg.nanosec = (int)((tick - msg.sec) * 1e9);

    // Publish the message
    tick_Pub->publish(msg);
}

void animStatePublisher::publishAnimStates(std::vector<double> stateVector, double simTime)
{
    if ((simTime - lastPubTime) >= dt)
    {
        // Publish body pose
        bodyPose_PFun(std::vector<double>(&stateVector[0], &stateVector[6]));

        // Publish motor angles
        motB_PFun(stateVector[7]);
        motC_PFun(stateVector[8]);
        motD_PFun(stateVector[9]);
        motE_PFun(stateVector[10]);

        // Publish simulation time
        tick_PFun(simTime);

        // Update lastPubTime
        lastPubTime = simTime;
    }
}