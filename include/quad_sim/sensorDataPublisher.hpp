// Add package headers

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#ifndef __SENSOR_DATA_PUBLISHER_HEADER__
#define __SENSOR_DATA_PUBLISHER_HEADER__

class sensorDataPublisher : public rclcpp::Node
{
public:
    // Contructor
    sensorDataPublisher(int64_t dtBat_ns, int64_t dtBaro_ns, int64_t dtImu_ns, int64_t dtMag_ns, int64_t dtGps_ns);

    // Publisher functions
    void batStat_PFun(double voltage, int64_t simTime_ns);
    void baro_PFun(double alt, int64_t simTime_ns);
    void imu_PFun(std::vector<double> stVect, std::vector<double> stDerVect, double g, int64_t simTime_ns);
    // void mag_PFun();
    // void gps_PFun();

private:
    // Restrict default Contructor
    sensorDataPublisher();

    // Publishing time in nanoseconds
    int64_t dtBat_ns, dtBaro_ns, dtImu_ns, dtMag_ns, dtGps_ns;

    // Variables for subscribers
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batStat_Pub;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr baro_Pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_Pub;
    // rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_Pub;
    // rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_msg;

    // Keep track of the simulation time for last publication, lpt = last publication time
    int64_t bat_lpt, baro_lpt, imu_lpt, mag_lpt, gps_lpt;
};

#endif // __SENSOR_DATA_PUBLISHER_HEADER__