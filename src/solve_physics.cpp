// Add Standard c++ headers
#include <iostream>

// Add package headers
#include <quad_sim/quadEomSystem.h>
#include <quad_sim/animStatePublisher.h>

int main(int argc, char **argv)
{
    // Some initialization.
    rclcpp::init(argc, argv);

    // Initialize the system
    quadEomSystem quad;

    // Initialize ODE solver
    boost::numeric::odeint::adams_bashforth_moulton<5, std::vector<double>> odeSolver;

    // Define clock and time-point variables
    std::chrono::high_resolution_clock solverClock;
    std::chrono::high_resolution_clock::time_point start, lastStart = solverClock.now();

    // Initialize the solver
    odeSolver.initialize(boost::numeric::odeint::runge_kutta_fehlberg78<std::vector<double>>(), quad, quad.q, quad.solverT, quad.solverDT);

    // Initialize the animStatePublisher node
    rclcpp::executors::MultiThreadedExecutor rosExecutor;
    // animStatePublisher animPubNode(0.01);
    // rclcpp::Node::SharedPtr animPubNodePtr = std::make_shared<animStatePublisher>(animPubNode);
    animStatePublisher::SharedPtr animPubNodePtr = std::make_shared<animStatePublisher>(0.01);
    rosExecutor.add_node(animPubNodePtr);

    while (rclcpp::ok())
    {
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

        // Publish states for animWindow to update the plot
        std::dynamic_pointer_cast<animStatePublisher>(animPubNodePtr)->publishAnimStates(quad.q, quad.solverT);

        // Allow ROS to finish publishing
        rosExecutor.spin_some();

        // Update the lastStart time-stamp
        lastStart = start;
    }

    // Shutdown the ROS executor
    rclcpp::shutdown();

    return 0;
}