#include <quad_sim/sensorDataPublisher.hpp>

sensorDataPublisher::sensorDataPublisher(int64_t dtBat_ns, int64_t dtBaro_ns, int64_t dtImu_ns, int64_t dtMag_ns, int64_t dtGps_ns) : Node("sensorDataPublisher"), dtBat_ns(dtBat_ns), dtBaro_ns(dtBaro_ns), dtImu_ns(dtImu_ns), dtMag_ns(dtMag_ns), dtGps_ns(dtGps_ns)
{
    // Initialize the Publishers
    batStat_Pub = this->create_publisher<sensor_msgs::msg::BatteryState>("batStat", rclcpp::SensorDataQoS());
    baro_Pub = this->create_publisher<sensor_msgs::msg::FluidPressure>("baro", rclcpp::SensorDataQoS());
    imu_Pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    // mag_Pub = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", rclcpp::SensorDataQoS());
    // gps_msg = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", rclcpp::SensorDataQoS());
}

void sensorDataPublisher::batStat_PFun(double voltage, int64_t simTime_ns)
{
    // Check if enough time has passed
    if ((simTime_ns - bat_lpt) >= dtBat_ns)
    {
        // Create message variable
        sensor_msgs::msg::BatteryState msg;

        // Add data to the message variable
        msg.header.stamp.sec = simTime_ns / 1000000000;
        msg.header.stamp.nanosec = simTime_ns % 1000000000;

        // Publish the message
        batStat_Pub->publish(msg);
    }
}

void sensorDataPublisher::baro_PFun(double alt, int64_t simTime_ns)
{
    // Check if enough time has passed
    if ((simTime_ns - baro_lpt) >= dtBaro_ns)
    {
        // Create message variable
        sensor_msgs::msg::FluidPressure msg;

        // Add data to the message variable
        msg.header.stamp.sec = simTime_ns / 1000000000;
        msg.header.stamp.nanosec = simTime_ns % 1000000000;

        // This implementation is based on https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html article.
        // Altitude is assumed to be in meters.

        // Define necessary variables
        double T, p; // T = Temperature (degC), p = Pressure (kPa)

        // Compute temperature and pressure based on the altitude
        if (alt <= 11000) // Troposphere, we should be mostly here
        {
            T = 15.04 - 0.00649 * alt;
            p = 101.29 * pow((T + 273.1) / 288.08, 5.256);
        }
        else if ((11000 < alt) && (alt <= 25000))
        {
            T = -56.46;
            p = 22.65 * exp(1.73 - 0.000157 * alt);
        }
        else
        {
            T = -131.21 + 0.00299 * alt;
            p = 2.488 * pow((T + 273.1) / 216.6, -11.388);
        }

        msg.fluid_pressure = p * 1000; // Pressure in Pascals.

        // Publish the message
        baro_Pub->publish(msg);
    }
}

std::vector<double> linAccInBodyFrame(const std::vector<double> &stVect, const std::vector<double> &stDerVect, const double &g)
{
    std::vector<double> accInBodyFrame(3);
    accInBodyFrame[0] = (pow(stVect[3], 2) + pow(stVect[4], 2) - pow(stVect[5], 2) - pow(stVect[6], 2)) * stDerVect[0] + (2 * (stVect[4] * stVect[5] - stVect[3] * stVect[6])) * stDerVect[1] + (2 * (stVect[4] * stVect[6] + stVect[3] * stVect[5])) * (stDerVect[2] - g);
    accInBodyFrame[1] = (2 * (stVect[4] * stVect[5] + stVect[3] * stVect[6])) * stDerVect[0] + (pow(stVect[3], 2) - pow(stVect[4], 2) + pow(stVect[5], 2) - pow(stVect[6], 2)) * stDerVect[1] + (2 * (stVect[5] * stVect[6] - stVect[3] * stVect[4])) * (stDerVect[2] - g);
    accInBodyFrame[2] = (2 * (stVect[4] * stVect[6] - stVect[3] * stVect[5])) * stDerVect[0] + (2 * (stVect[5] * stVect[6] + stVect[3] * stVect[4])) * stDerVect[1] + (pow(stVect[3], 2) - pow(stVect[4], 2) - pow(stVect[5], 2) + pow(stVect[6], 2)) * (stDerVect[2] - g);
    return accInBodyFrame;
}

void sensorDataPublisher::imu_PFun(std::vector<double> stVect, std::vector<double> stDerVect, double g, int64_t simTime_ns)
{
    // Check if enough time has passed
    if ((simTime_ns - bat_lpt) >= dtBat_ns)
    {
        // Create message variable
        sensor_msgs::msg::Imu msg;

        // Add data to the message variable
        msg.header.stamp.sec = simTime_ns / 1000000000;
        msg.header.stamp.nanosec = simTime_ns % 1000000000;

        msg.orientation.w = stVect[3];
        msg.orientation.x = stVect[4];
        msg.orientation.y = stVect[5];
        msg.orientation.z = stVect[6];

        msg.angular_velocity.x = stVect[14];
        msg.angular_velocity.y = stVect[15];
        msg.angular_velocity.z = stVect[16];

        // Get linear acceleration in body-attached frame
        std::vector<double> linAcc = linAccInBodyFrame(stVect, stDerVect, g);
        msg.linear_acceleration.x = linAcc[0];
        msg.linear_acceleration.y = linAcc[1];
        msg.linear_acceleration.z = linAcc[2];

        // Publish the message
        imu_Pub->publish(msg);
    }
}