#include <quad_sim/sensorDataPublisher.hpp>

sensorDataPublisher::sensorDataPublisher(int64_t dtBat_ns, int64_t dtBaro_ns, int64_t dtImu_ns) : Node("sensorDataPublisher"), dtBat(dtBat_ns / 1000000000, dtBat_ns % 1000000000), dtBaro(dtBaro_ns / 1000000000, dtBaro_ns % 1000000000), dtImu(dtImu_ns / 1000000000, dtImu_ns % 1000000000), bat_lpt(int64_t(0), RCL_ROS_TIME), baro_lpt(bat_lpt), imu_lpt(bat_lpt)
{
    // Initialize the Publishers
    batStat_Pub = this->create_publisher<sensor_msgs::msg::BatteryState>("batStat", rclcpp::SensorDataQoS());
    baro_Pub = this->create_publisher<sensor_msgs::msg::FluidPressure>("baro", rclcpp::SensorDataQoS());
    imu_Pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
}

void sensorDataPublisher::batStat_PFun(double voltage)
{
    // Get current time
    rclcpp::Time timeNow = now();

    // Check if enough time has passed
    if ((bat_lpt + dtBat) <= timeNow)
    {
        // Create message variable
        sensor_msgs::msg::BatteryState msg;

        // Add data to the message variable
        msg.header.stamp = timeNow;
        msg.voltage = voltage;

        // Publish the message
        batStat_Pub->publish(msg);

        // Update lpt
        bat_lpt = timeNow;
    }
}

void sensorDataPublisher::baro_PFun(double alt)
{
    // Get current time
    rclcpp::Time timeNow = now();

    // Check if enough time has passed
    if ((baro_lpt + dtBaro) <= timeNow)
    {
        // Create message variable
        sensor_msgs::msg::FluidPressure msg;

        // Add data to the message variable
        msg.header.stamp = timeNow;

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

        // Update lpt
        baro_lpt = timeNow;
    }
}

std::vector<double> linAccInBodyFrame(const std::vector<double> &stVect, const std::vector<double> &stDerVect, const double &g)
{
    Eigen::Matrix3d R_NA;
    Eigen::Vector3d linAcc;

    // Copy linear acceleration values
    linAcc << stDerVect[11], stDerVect[12], stDerVect[13] - g;

    // Define the ortation matrix
    R_NA << pow(stVect[3], 2) + pow(stVect[4], 2) - pow(stVect[5], 2) - pow(stVect[6], 2), 2 * (stVect[4] * stVect[5] - stVect[3] * stVect[6]), 2 * (stVect[4] * stVect[6] + stVect[3] * stVect[5]),
        2 * (stVect[4] * stVect[5] + stVect[3] * stVect[6]), pow(stVect[3], 2) - pow(stVect[4], 2) + pow(stVect[5], 2) - pow(stVect[6], 2), 2 * (stVect[5] * stVect[6] - stVect[3] * stVect[4]),
        2 * (stVect[4] * stVect[6] - stVect[3] * stVect[5]), 2 * (stVect[5] * stVect[6] + stVect[3] * stVect[4]), pow(stVect[3], 2) - pow(stVect[4], 2) - pow(stVect[5], 2) + pow(stVect[6], 2);
    R_NA = R_NA / (pow(stVect[3], 2) + pow(stVect[4], 2) + pow(stVect[5], 2) + pow(stVect[6], 2));

    // Apply the rotation
    linAcc = R_NA.transpose() * linAcc;

    // Return the value
    return std::vector<double>(linAcc.data(), linAcc.data() + linAcc.rows() * linAcc.cols());
}

void sensorDataPublisher::imu_PFun(std::vector<double> stVect, std::vector<double> stDerVect, double g)
{
    // Get current time
    rclcpp::Time timeNow = now();

    // Check if enough time has passed
    if ((imu_lpt + dtImu) <= timeNow)
    {
        // Create message variable
        sensor_msgs::msg::Imu msg;

        // Add data to the message variable
        msg.header.stamp = timeNow;

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

        // Update last publication time
        imu_lpt = timeNow;
    }
}