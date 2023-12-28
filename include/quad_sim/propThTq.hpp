#include <iostream>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <numeric>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>

#ifndef __PROP_TH_TQ_HEADER__
#define __PROP_TH_TQ_HEADER__

void appendStDataFromFile(const std::filesystem::path fPath, std::vector<double> &J, std::vector<double> &Ct, std::vector<double> &Cp);

void appendDynDataFromFile(const std::vector<std::filesystem::path> &fPath, std::vector<double> &J, std::vector<double> &Ct, std::vector<double> &Cp);

void splitDatasetFiles(std::filesystem::path datasetDir, std::filesystem::path &stDataFile, std::vector<std::filesystem::path> &dynDataFiles);

void fitPropData(std::filesystem::path propDataDir);

double getAirDensity(double alt);

void getPropThTq(double u, double rho, double propVel, double propDia, double g, double &th, double &tq);

#endif // __PROP_TH_TQ_HEADER__