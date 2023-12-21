// Add Standard c++ headers
#include <iostream>

// Add package headers
#include <quad_sim/quadEomSystem.h>

int main(int argc, char **argv)
{
    // Initialize the system
    quadEomSystem quad;

    // Initialize ODE solver
    boost::numeric::odeint::adams_bashforth_moulton<5, std::vector<double>> odeSolver;

    std::chrono::high_resolution_clock solverClock;

    // Initialize the solver
    odeSolver.initialize(boost::numeric::odeint::runge_kutta_fehlberg78<std::vector<double>>(), quad, quad.q, quad.solverT, quad.solverDT);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (int i = 0; i < 10000; ++i)
    {
        std::chrono::high_resolution_clock::time_point start = solverClock.now();
        odeSolver.do_step(quad, quad.q, quad.solverT, quad.solverDT);
        quad.solverT += quad.solverDT;
        std::cout << i << '\t' << quad.q[7] << '\t' << quad.q[8] << '\t' << quad.q[9] << '\t' << quad.q[10] << std::endl;
        while (std::chrono::duration_cast<std::chrono::duration<double>>(solverClock.now() - start).count() < 0.001);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
}