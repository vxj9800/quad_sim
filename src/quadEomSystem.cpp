// Add package headers
#include <quad_sim/quadEomSystem.h>

quadEomSystem::quadEomSystem()
{
    // Choose initial time
    solverT = 0;

    // Define System Initial condition
    q = {0, 0, 0, 1, 0, 0, 0, -M_PI_4, M_PI_4, -M_PI_4, M_PI_4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
    RHS = (COEF.transpose() * COEF).ldlt().solve(COEF.transpose() * RHS);
    dx.assign(RHS.begin(), RHS.end());
    // std::cout << "q:\t";
    // for (size_t i = 0; i < (size_t)RHS.rows(); ++i)
        // std::cout << dx[i] << '\t';
    // std::cout << std::endl;
}