// Add package headers
#include <quad_sim/quadEomSystem.hpp>

quadEomSystem::quadEomSystem()
{
    // Define initial time
    solverT_ns = 0; // ns
    
    // Define time-step
    solverDT_ns = 1000000; // 1ms

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
    RHS = (COEF.transpose() * COEF).ldlt().solve(COEF.transpose() * RHS); // Uses LU decomposition
    // RHS = COEF.fullPivHouseholderQr().solve(RHS); // Uses QR decomposition
    // RHS = COEF.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(RHS); // Uses SVD decomposition
    dx.assign(RHS.begin(), RHS.end());
    // std::cout << "q:\t";
    // for (size_t i = 0; i < (size_t)RHS.rows(); ++i)
        // std::cout << dx[i] << '\t';
    // std::cout << std::endl;
}

double& quadEomSystem::getSolverT()
{
    t_s = solverT_ns * 1e-9;
    return t_s;
}

double& quadEomSystem::getSolverDT()
{
    dt_s = solverDT_ns * 1e-9;
    return dt_s;
}