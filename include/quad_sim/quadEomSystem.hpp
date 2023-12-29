// Add Standard C++ headers
#include <chrono>

// Add package headers
extern "C"
{
#include <eomCoef.h>
#include <eomRhs.h>
#include <propVelocities.h>
}

// Add ROS libraries
#include <builtin_interfaces/msg/time.hpp>

// Add other external libraries
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>

#ifndef __QUAD_EOM_SOLVER_HEADER__
#define __QUAD_EOM_SOLVER_HEADER__

// Model Parameter Samples // %
// Frame : https://speedyfpv.com/products/truexs-stretched-220mm-fpv-racing-drone-frame-kit-for-5-propellers?variant=39876517363885
// Motors : https://shop.iflight-rc.com/xing-2205-fpv-nextgen-motor-black-pro1521?search=2205
// Battery : https://www.amazon.com/Socokin-Battery-3300mAh-Vehicles-Trucks/dp/B086GTSPQN/ref=sr_1_28?crid=21QGKY7B7764X&keywords=6s+lipo&qid=1686777293&sprefix=6s+lipo%2Caps%2C142&sr=8-28&ufe=app_do%3Aamzn1.fos.18630bbb-fcbb-42f8-9767-857e17e03685
// Propellers : https://www.getfpv.com/lumenier-5x3-5-2-blade-propeller-set-of-4-green.html

class quadEomSystem
{
private:
public:
    quadEomSystem();
    void operator()(const std::vector<double> &x, std::vector<double> &dx, const double t);
    double& getSolverT();
    double& getSolverDT();
    void incrmntTime(builtin_interfaces::msg::Time dt);

    // Define state vector
    std::vector<double> q = std::vector<double>(21);

    // Constants for motor positions
    std::vector<double> pB = {0.08, 0.08, 0.015};   // pB = [lB; wB; hB];
    std::vector<double> pC = {-0.08, 0.08, 0.015};  // pC = [lC; wC; hC];
    std::vector<double> pD = {-0.08, -0.08, 0.015}; // pD = [lD; wD; hD];
    std::vector<double> pE = {0.08, -0.08, 0.015};  // pE = [lE; wE; hE];

    // Mass and Inertia values
    double g = 9.81; // Gravitational constant
    std::vector<double> mVals = {
        // Mass of bodies
        0.155 + 4 * (0.021 * 0.9) + 0.5 + 0.1, // A = Frame + 4*Motor Stator + Battery + Circuit boards
        (0.021 * 0.1) + 0.0028,                // B = Motor rotor + Prop
        (0.021 * 0.1) + 0.0028,                // C = Motor rotor + Prop
        (0.021 * 0.1) + 0.0028,                // D = Motor rotor + Prop
        (0.021 * 0.1) + 0.0028                 // E = Motor rotor + Prop
    };

    // A_I_AA = diag([1/12*0.5*(0.042^2 + 0.043^2),... % Inertia dyadic of the battery
    //            1/12*0.5*(0.137^2 + 0.043^2),... % as a cuboid of dimension 137mm X 42mm X 43mm
    //            1/12*0.5*(0.137^2 + 0.042^2)])...
    //     +simprot(3,pi/4)...
    //     *diag([1/12*0.0775*(0.030^2 + 0.005^2),... % Inertia dyadic of half the frame
    //            1/12*0.0775*(0.220^2 + 0.005^2),... % as a cuboid of dimension 220mm X 30mm X 5mm
    //            1/12*0.0775*(0.220^2 + 0.020^2)])...
    //     *simprot(3,pi/4)'...
    //     +simprot(3,-pi/4)...
    //     *diag([1/12*0.0775*(0.030^2 + 0.005^2),... % Inertia dyadic of half the frame
    //            1/12*0.0775*(0.220^2 + 0.005^2),... % as a cuboid of dimension 220mm X 30mm X 5mm
    //            1/12*0.0775*(0.220^2 + 0.020^2)])...
    //     *simprot(3,-pi/4)';
    std::vector<double> IA = {0.000469260416667, 0.001177802083333, 0.001485875000000, 0, 0, 0}; // [IAxx; IAyy; IAzz; IAxy; IAyz; IAzx]

    // B_I_BB = diag([1/12*(0.021*0.1)*(3*(0.0286^2 + 0.025^2) + 0.01^2),... % Inertia dyadic of motor rotor as a hollow cylinder with
    //            1/12*(0.021*0.1)*(3*(0.0286^2 + 0.025^2) + 0.01^2),... % outer radius of 28.1mm and inner of 25mm and height of 10mm
    //            1/2*(0.021*0.1)*(0.0286^2 + 0.025^2)])...
    //     +diag([1/12*0.0028*(0.001^2 + 0.01^2),... % Inertia dyadic of propeller
    //            1/12*0.0028*(0.001^2 + 0.127^2),... % as a cuboid of dimension 127mm X 10mm X 1mm
    //            1/12*0.0028*(0.01^2 + 0.127^2)]);
    std::vector<double> IB = {0.079862066666667e-6, 0.453872066666667e-6, 0.530187466666667e-6, 0, 0, 0}; // [IBxx; IByy; IBzz; IBxy; IByz; IBzx]
    std::vector<double> IC = IB, ID = IB, IE = IB;

    // Define Propeller data
    double propDia = 0.127; // Propeller Diameter

    // Motor Properties // %
    double Vllmax = 16.8;    // Max line to line voltage for the motor
    double motRll = 0.07241; // Line to line resistance
    double motKv = 2400;     // Kv rating of the motor, i.e.rpm / V

    // Define containers for propeller thrust and motor torque values
    std::vector<double> fVals = {0, 0, 0, 0}; // Will come from controller subscribers
    std::vector<double> tVals = {0, 0, 0, 0}; // Will come from controller subscribers

    // Integration parameters
    builtin_interfaces::msg::Time solverT; // The ROS message is used here to avoid problems associated with C++ floatingpoint value comparisons
    builtin_interfaces::msg::Time solverDT; // Ode solver time step in seconds
    double t, dt; // Temporary variables to allow getSolverT and getSolverDt return an lvalue/double&
};

#endif // __QUAD_EOM_SOLVER_HEADER__