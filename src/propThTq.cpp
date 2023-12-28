#include <quad_sim/propThTq.hpp>

// Define local static variable to store propeller thrust and torque curve fit parameters
static Eigen::VectorXd ctFitParams(3), cpFitParams(3);

void appendStDataFromFile(const std::filesystem::path fPath, std::vector<double> &J, std::vector<double> &Ct, std::vector<double> &Cp)
{
    // Open the file
    std::ifstream stFile(fPath);

    // Find the number of data rows
    size_t nLines = std::count(std::istreambuf_iterator<char>(stFile), std::istreambuf_iterator<char>(), '\n') - 1;

    // Reset seek to the first data row
    stFile.clear();
    stFile.seekg(0);
    stFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the first line

    // Create storage variables
    std::vector<double> stCt(nLines), stCp(nLines);

    // Copy the data
    for (size_t i = 0; i < nLines; ++i)
    {
        double val;
        stFile >> val; // Waste RPM value
        stFile >> val;
        stCt[i] = val;
        stFile >> val;
        stCp[i] = val / 2 / M_PI;
    }

    // Calculate mean of Ct and Cp values
    J.push_back(0);
    Ct.push_back(std::accumulate(stCt.begin(), stCt.end(), 0.0) / stCt.size());
    Cp.push_back(std::accumulate(stCp.begin(), stCp.end(), 0.0) / stCp.size());
}

void appendDynDataFromFile(const std::vector<std::filesystem::path> &fPath, std::vector<double> &J, std::vector<double> &Ct, std::vector<double> &Cp)
{
    for (size_t i = 0; i < fPath.size(); ++i)
    {
        // Open the file
        std::ifstream dynFile(fPath[i]);

        // Find the number of data rows
        size_t nLines = std::count(std::istreambuf_iterator<char>(dynFile), std::istreambuf_iterator<char>(), '\n') - 1;

        // Reset seek to the first data row
        dynFile.clear();
        dynFile.seekg(0);
        dynFile.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore the first line

        // Create local storage
        std::vector<double> dynJ(nLines), dynCt(nLines), dynCp(nLines);

        // Copy the data
        for (size_t i = 0; i < nLines; ++i)
        {
            double val;
            dynFile >> val;
            dynJ[i] = val;
            dynFile >> val;
            dynCt[i] = val;
            dynFile >> val;
            dynCp[i] = val / 2 / M_PI;
            dynFile >> val; // Waste ETA value
        }

        J.insert(J.end(), dynJ.begin(), dynJ.end());
        Ct.insert(Ct.end(), dynCt.begin(), dynCt.end());
        Cp.insert(Cp.end(), dynCp.begin(), dynCp.end());
    }
}

void splitDatasetFiles(std::filesystem::path datasetDir, std::filesystem::path &stDataFile, std::vector<std::filesystem::path> &dynDataFiles)
{
    for (std::filesystem::directory_entry file : std::filesystem::directory_iterator(datasetDir))
    {
        // Get the file name only, without extension
        std::string fName = file.path().stem();

        // Split the file name by '_' as delimiter
        std::vector<std::string> fSplit;
        boost::algorithm::split(fSplit, fName, boost::is_any_of("_"));

        // Check if the file contains static or dynamic data
        if (fSplit[2] == "geom")                 // If the file name contains "geom"
            continue;                            // then discard it
        else if (fSplit[2] == "static")          // If the file name contains "static"
            stDataFile = file.path();            // then store its path as a static data file
        else                                     // Otherwise
            dynDataFiles.push_back(file.path()); // and store its path as a dynamic data file
    }
}

void fitPropData(std::filesystem::path propDataDir)
{
    // Split the files into dynamic and static datasets
    std::filesystem::path stDataFile;
    std::vector<std::filesystem::path> dynDataFiles;
    splitDatasetFiles(propDataDir, stDataFile, dynDataFiles);

    // Create containers for J, Ct, Cp
    std::vector<double> J, Ct, Cp;

    // Get static data
    appendStDataFromFile(stDataFile, J, Ct, Cp);

    // Get Dynamic data
    appendDynDataFromFile(dynDataFiles, J, Ct, Cp);

    // Fit the data to a0 + a1*J + a2*J^2
    // The curve-fit can be performed by
    // solving matrix equatiion:
    // \mathbf{y} = X*\mathbf{a}
    // where, \mathbf{y} is the vector of output values, with elements y_i.
    // X is the polynomial matrix with elements x_ij = (x_i)^j.
    // \mathbf{a} is the vector of coefficients, with elements a_j.
    // i is the data point iterator, starting from 0.
    // j is the exponent iterator, starting from 0.
    Eigen::VectorXd y(J.size());
    Eigen::MatrixXd X(J.size(), 3);

    // Compute polynomial matrix
    for (size_t i = 0; i < (size_t)y.size(); ++i)
        for (size_t j = 0; j < (size_t)ctFitParams.size(); ++j)
            X(i, j) = pow(J[i], j);

    // Fit the Ct data
    for (size_t i = 0; i < (size_t)y.size(); ++i)
        y(i) = Ct[i];
    ctFitParams = X.fullPivHouseholderQr().solve(y);

    // Fit the Cp data
    for (size_t i = 0; i < (size_t)y.size(); ++i)
        y(i) = Cp[i];
    cpFitParams = X.fullPivHouseholderQr().solve(y);
}

double getAirDensity(double alt)
{
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
    return p / (0.2869 * (T + 273.1));
}

void getPropThTq(double u, double alt, double propVel, double propDia, double g, double &th, double &tq)
{
    // u = Angular velocity of the motor/propeller in rad/sec
    // alt = Altitude of the propeller in meters.
    // propVel = Translational velocity of the propeller in the direction of
    // the axis of rotation in m/s.
    // propDia = Diameter of the propeller in m.
    // g = Gravitational acceleration in m/s^2.
    // th = Computed propeller thrust in N.
    // tq = Computed propeller torque in Nm.

    // This function depends on the UIUC dataset. First, a second order
    // polynomial is fitted to the thrust coefficient and torque coefficient
    // data. Those curve-fit coefficients are used in this function to
    // estimate the C_T and C_Q values based on the advancement ratio (J). From
    // there, the thrust and the torque is calculated. One assumption here is
    // that the air is stationary at all times. However, it should be easy to
    // account for the air velocity as well. It is also assumed that the
    // propeller rotates in such a way that it creates thrust to lift the
    // quadcopter.

    // Define necessary variables
    double J;

    // Calculate air density at alt
    double rho = getAirDensity(alt); // Air density in kg/m^3

    // Define conversion factor for rad/sec to revs/sec
    const double radpsec2revpsec = 0.159154943;
    

    // Return zero torque and thrust if the props are not rotating
    if (u == 0)
    {
        th = 0;
        tq = 0;
        return;
    }

    // Technically the motors rotate in both CC and CCW directions,
    // and based on the propeller type, either rotation can produce
    // positive thrust. However, here it is assumed that the motor
    // rotation and propeller type is matched correctly so that the
    // thrust produced will always lift the propeller.
    u = abs(u);

    // Calculate advancement ratio
    if (propVel < 0) 
        J = 0;
    else
        J = propVel / (u * radpsec2revpsec) / propDia;

    // Calculate thrust and torque coefficient from curve-fit coefficients
    // The coefficients should follow format given below
    // y = coef(0)*x^0 + coef(1)*x^1 + coef(3)*x^2
    double C_T = ctFitParams(0) + ctFitParams(1) * J + ctFitParams(2) * J * J;
    double C_Q = cpFitParams(0) + cpFitParams(1) * J + cpFitParams(2) * J * J;

    // Calculate the thrust and the torque
    th = C_T * rho * pow(u * radpsec2revpsec,2) * pow(propDia, 4);
    tq = C_Q * rho * pow(u * radpsec2revpsec,2) * pow(propDia, 5);

    // Convert kgf unit to N unit
    th = th * g;
    tq = tq * g;
}