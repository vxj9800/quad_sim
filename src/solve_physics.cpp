// Add Standard c++ headers
#include <iostream>

// Add ROS headers
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// Add package headers
#include <quad_sim/quadEomSystem.h>
#include <quad_sim/animStatePublisher.h>
#include <quad_sim/propThTq.h>

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
    odeSolver.initialize(boost::numeric::odeint::runge_kutta_fehlberg78<std::vector<double>>(), quad, quad.q, quad.solverT, quad.solverDT);

    // Initialize propeller thrust and torque model
    fitPropData(pkgShareDir + "/propData");

    // Initialize the ROS executor
    rclcpp::executors::MultiThreadedExecutor rosExecutor;

    // Get a shared pointer for a node object
    std::shared_ptr<animStatePublisher> animPubNodePtr = std::make_shared<animStatePublisher>(0.01);
    rosExecutor.add_node(animPubNodePtr);

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
            getPropThTq(quad.q[17 + i], 1, propVels(i), quad.propDia, quad.g, th, tq); // Thrust and torque on motor B
            quad.fVals[i] = th;
            quad.tVals[i] = -tq;
        }

        // Get control signal, this will eventually be a blocking call if the system is supposed to run in lockstep with the GNC loop
        // Apply motor torques
        for (size_t i = 0; i < 4; ++i)
            quad.tVals[i] += 0.0175;

        // Wait till the real time equal to solver step size has passed
        while(std::chrono::duration_cast<std::chrono::duration<double>>((start = solverClock.now()) - lastStart).count() < quad.solverDT);

        // Integrate the system by one step
        odeSolver.do_step(quad, quad.q, quad.solverT, quad.solverDT);
        quad.solverT += quad.solverDT;

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

        // // Publish sensor values for GNC node to return control values on time
        // animPubNodePtr->publishAnimStates(quad.q, quad.solverT);

        // // Allow ROS to finish publishing
        // rosExecutor.spin_some();

        // Publish states for animWindow to update the plot
        animPubNodePtr->publishAnimStates(quad.q, quad.solverT);

        // Allow ROS to finish publishing
        rosExecutor.spin_some();

        // Update the lastStart time-stamp
        lastStart = start;
    }

    // Shutdown the ROS executor
    rclcpp::shutdown();

    return 0;
}