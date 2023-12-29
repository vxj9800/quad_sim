// Add package headers
#include <quad_sim/quadEomSystem.hpp>

quadEomSystem::quadEomSystem()
{
    // Choose initial time
    solverT.sec = 0;
    solverT.nanosec = 0;

    // Define time-step
    solverDT.sec = 0;
    solverDT.nanosec = 1000000; // 1ms

    // Define System Initial condition
    q = {0, 0, 0, 1, 0, 0, 0, -M_PI_4, M_PI_4, -M_PI_4, M_PI_4, 0, 0, 0, 0, 0, 0, -2400 / 60 * 2 * M_PI, 2400 / 60 * 2 * M_PI, -2400 / 60 * 2 * M_PI, 2400 / 60 * 2 * M_PI};
}

void quadEomSystem::operator()(const std::vector<double> &x, std::vector<double> &dx, const double t)
{
    // Get state variables
    std::vector<double> st = x;

    // Define other matrices and vectors
    Eigen::MatrixXd COEF(21, 21);
    Eigen::VectorXd RHS(21);

    eomCoef(st.data(), g, pB.data(), pC.data(), pD.data(), pE.data(), mVals.data(), IA.data(), IB.data(), IC.data(), ID.data(), IE.data(), fVals.data(), tVals.data(), COEF.data());

    // Get full RHS
    eomRhs(st.data(), g, pB.data(), pC.data(), pD.data(), pE.data(), mVals.data(), IA.data(), IB.data(), IC.data(), ID.data(), IE.data(), fVals.data(), tVals.data(), RHS.data());

    // Compute state derivatives
    COEF.transposeInPlace();
    // RHS = (COEF.transpose() * COEF).ldlt().solve(COEF.transpose() * RHS); // Uses LU decomposition
    RHS = COEF.fullPivHouseholderQr().solve(RHS); // Uses QR decomposition
    dx.assign(RHS.begin(), RHS.end());
    // std::cout << "q:\t";
    // for (size_t i = 0; i < (size_t)RHS.rows(); ++i)
        // std::cout << dx[i] << '\t';
    // std::cout << std::endl;
}

double& quadEomSystem::getSolverT()
{
    t = solverT.sec + solverT.nanosec * 1e-9;
    return t;
}

double& quadEomSystem::getSolverDT()
{
    dt = solverDT.sec + solverDT.nanosec * 1e-9;
    return dt;
}

void quadEomSystem::incrmntTime(builtin_interfaces::msg::Time dt)
{
    solverT.sec += dt.sec;
    solverT.nanosec += dt.nanosec;
    solverT.sec += solverT.nanosec / 1000000000;
    solverT.nanosec = solverT.nanosec % 1000000000;
}