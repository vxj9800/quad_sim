// Add Standard c++ headers
#include <iostream>

// Add ROS headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Add package headers
#include <quad_sim/quadEomSystem.hpp>
#include <quad_sim/animStatePublisher.hpp>
#include <quad_sim/sensorDataPublisher.hpp>
#include <quad_sim/controllerInSubscriber.hpp>
#include <quad_sim/propThTq.hpp>
#include <quad_sim/motorTq.hpp>

// If the package name is not defined at compile time then set it to empty
#ifndef ROS_PACKAGE_NAME
#define ROS_PACKAGE_NAME ""
#endif

int main(int argc, char **argv)
{
    // Some initialization.
    rclcpp::init(argc, argv);

    // Get location of the package share directory
    std::string pkgShareDir = ament_index_cpp::get_package_share_directory(ROS_PACKAGE_NAME);

    // Initialize the system
    quadEomSystem quad;

    // Initialize ODE solver
    boost::numeric::odeint::adams_bashforth_moulton<5, std::vector<double>> odeSolver;

    // Define clock and time-point variables
    std::chrono::high_resolution_clock solverClock;
    std::chrono::high_resolution_clock::time_point start, lastStart = solverClock.now();

    // Initialize the solver
    odeSolver.initialize(boost::numeric::odeint::runge_kutta_fehlberg78<std::vector<double>>(), quad, quad.q, quad.getSolverT(), quad.getSolverDT());

    // Initialize propeller thrust and torque model
    fitPropData(pkgShareDir + "/propData");

    // Initialize the ROS executor
    rclcpp::executors::MultiThreadedExecutor rosExecutor;

    // Get a shared pointer for animation node object
    std::shared_ptr<animStatePublisher> animPubNodePtr = std::make_shared<animStatePublisher>(10000000); // 10ms
    rosExecutor.add_node(animPubNodePtr);

    // Get a shared pointer for sensors node object
    std::shared_ptr<sensorDataPublisher> sensPubNodePtr = std::make_shared<sensorDataPublisher>(1000000000, 10000000, 500000, 500000, 100000000); // 10ms
    rosExecutor.add_node(sensPubNodePtr);

    // Get a shared pointer for controller input node object
    std::shared_ptr<controllerInSubscriber> cntrlInSubNodePtr = std::make_shared<controllerInSubscriber>();
    rosExecutor.add_node(cntrlInSubNodePtr);

    while (rclcpp::ok())
    // for (size_t i = 0; i < 10; ++i)
    {
        // Calculate propeller velocities
        Eigen::VectorXd propVels(4);
        propVelocities(quad.q.data(), quad.pB.data(), quad.pC.data(), quad.pD.data(), quad.pE.data(), propVels.data());

        // Compute propeller force and torques
        double th, tq;
        for (size_t i = 0; i < 4; ++i)
        {
            getPropThTq(quad.q[17 + i], quad.q[2], propVels(i), quad.propDia, quad.g, th, tq); // Thrust and torque on motor B
            quad.fVals[i] = th;
            quad.tVals[i] = -tq;
        }

        // Get control signal, motor voltages from esc signals
        std::vector<double> motVolts(4);
        cntrlInSubNodePtr->getMotVolts(quad.battVolts, motVolts);

        // Apply motor torques based on the calculated motor voltage
        for (size_t i = 0; i < 4; ++i)
            quad.tVals[i] += motTq(quad.q[17 + i], motVolts[i], quad.motRll, quad.motKv);

        // Get the arming state of the quadcopter
        int64_t armStateTS;
        bool quadArmed = cntrlInSubNodePtr->getArmState(armStateTS);

        // Wait till the real time equal to solver step size has passed
        while(std::chrono::duration_cast<std::chrono::duration<double>>((start = solverClock.now()) - lastStart).count() < quad.getSolverDT());

        // Integrate the system by one step only if the system is armed
        if (quadArmed)
        {
            odeSolver.do_step(quad, quad.q, quad.getSolverT(), quad.getSolverDT());
            quad.solverT_ns += quad.solverDT_ns;
        }

        // Normalize the euler parameters
        double eNorm = sqrt(pow(quad.q[3], 2) + pow(quad.q[4], 2) + pow(quad.q[5], 2) + pow(quad.q[6], 2));
        quad.q[3] /= eNorm;
        quad.q[4] /= eNorm;
        quad.q[5] /= eNorm;
        quad.q[6] /= eNorm;

        // Wrap the motor angles to 2*pi
        quad.q[7] = fmod(quad.q[7], 2 * M_PI);
        quad.q[8] = fmod(quad.q[8], 2 * M_PI);
        quad.q[9] = fmod(quad.q[9], 2 * M_PI);
        quad.q[10] = fmod(quad.q[10], 2 * M_PI);

        // Publish sensor values for GNC node to return control values on time
        quad(quad.q, quad.qd, quad.getSolverT()); // Get state derivatives
        sensPubNodePtr->baro_PFun(quad.q[2], quad.solverT_ns); // Publish baro data
        sensPubNodePtr->imu_PFun(quad.q, quad.qd, quad.g, quad.solverT_ns); // Publish imu data

        // Allow ROS to finish publishing
        rosExecutor.spin_some();

        // Publish states for animWindow to update the plot
        animPubNodePtr->publishAnimStates(quad.q, quad.solverT_ns);

        // Allow ROS to finish publishing
        rosExecutor.spin_some();

        // Update the lastStart time-stamp
        lastStart = start;
    }

    // Shutdown the ROS executor
    rclcpp::shutdown();

    return 0;
}