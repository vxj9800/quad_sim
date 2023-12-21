// Add package headers
#include <quad_sim/quadEomSystem.h>

quadEomSystem::quadEomSystem()
{
    // Choose initial time
    solverT = 0;

    // Define System Initial condition
    q = {0, 0, 0, 1, 0, 0, 0, -M_PI_4, M_PI_4, -M_PI_4, M_PI_4, 0, 0, 0, 0, 0, 0, M_PI / 10, M_PI / 10, M_PI / 10, M_PI / 10};
}

void quadEomSystem::operator()(const std::vector<double> &x, std::vector<double> &dx, const double t)
{
    // Get state variables
    std::vector<double> st = x;

    // Get full mass matrix
    std::vector<double> coef(441);
    eomCoef(st.data(), g, pB.data(), pC.data(), pD.data(), pE.data(), mVals.data(), IA.data(), IB.data(), IC.data(), ID.data(), IE.data(), fVals.data(), tVals.data(), coef.data());

    // Get full RHS
    std::vector<double> rhs(21);
    eomRhs(st.data(), g, pB.data(), pC.data(), pD.data(), pE.data(), mVals.data(), IA.data(), IB.data(), IC.data(), ID.data(), IE.data(), fVals.data(), tVals.data(), rhs.data());

    // Invert the mass matrix
    boost::numeric::ublas::matrix<double> mass(21, 21);
    for (int i = 0; i < 21; ++i)
        for (int j = 0; j < 21; ++j)
            mass(i, j) = coef[i * 21 + j];
    
    // Compute state derivatives
    for (int i = 0; i < 11; ++i)
        dx[i] = rhs[i];
    for (int i = 11; i < 21; ++i)
        dx[i] = 0;
}