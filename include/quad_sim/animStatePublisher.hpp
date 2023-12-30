// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <builtin_interfaces/msg/time.hpp>

#ifndef __ANIM_STATE_PUBLISHER_HEADER__
#define __ANIM_STATE_PUBLISHER_HEADER__

class animStatePublisher : public rclcpp::Node
{
public:
    // Contructor
    animStatePublisher(int64_t dt_ns);

    // Single function to publish everything
    void publishAnimStates(std::vector<double> stateVector, int64_t simTime_ns);

private:
    // Restrict default Contructor
    animStatePublisher();

    // Publishing time in nanoseconds
    int64_t dt_ns = 0;

    // Variables for subscribers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr bodyPose_Pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motB_Pub, motC_Pub, motD_Pub, motE_Pub;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr tick_Pub;

    // Publisher functions
    void bodyPose_PFun(std::vector<double> bodyPose);
    void motB_PFun(double motB);
    void motC_PFun(double motC);
    void motD_PFun(double motD);
    void motE_PFun(double motE);
    void tick_PFun(int64_t tick_ns);

    // Time in nanoseconds when data was last published
    int64_t lastPubTime_ns = 0;
};

#endif // __ANIM_STATE_PUBLISHER_HEADER__