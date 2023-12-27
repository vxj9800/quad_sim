import glob
import math
from scipy.optimize import curve_fit
import numpy
import sys
import getopt
import os


# Function to parse each line in the data files
def myParser(myString):
    lastChar = ' '
    words = []
    for c in myString:
        if lastChar == ' ' and c == ' ':
            continue
        elif lastChar == ' ' and c != ' ':
            words.append(c)
        elif c != ' ' and c != '\n':
            words[-1] = words[-1] + c
        lastChar = c
    return words


# Functiuon to fit to the dataset
def fitFun(J, p0, p1, p2):
    return p0 + p1*J + p2*J**2


def main(argv):

    if not argv:
        print("Path to the propData is not provided, cannot continue.")
        return
    else:
        try:
            propDirPath = argv[0]
            os.chdir(propDirPath)
        except FileNotFoundError:
            print("Directory: {0} does not exist".format(propDirPath))
            return
        except NotADirectoryError:
            print("{0} is not a directory".format(propDirPath))
            return
        except PermissionError:
            print("You do not have permissions to change to {0}".format(propDirPath))
            return
    
    # Check if the parameters are already available
    if os.path.exists("./paramsCt.csv") and os.path.exists("./paramsCq.csv"):
        print("Curve-fit coefficients are already calculated, recomputation is avoided.")
        return

    # Get the list of files in the directory
    files = glob.glob("*.txt", recursive=False)

    # Split the files into dynamic and static datasets
    stDatFile = []
    dynDatFiles = []
    rpmVals = []
    for f in files:
        fNameSplit = f.split('_', 3)
        if fNameSplit[2] == "geom.txt":
            continue
        elif fNameSplit[2] == "static.txt":
            stDatFile.append(f)
        else:
            rpmVals.append(fNameSplit[2].split('.')[0])
            dynDatFiles.append(f)

    # Create variables to store the data
    J, Cq, Ct = [], [], []

    # Load the static data first
    dataFile = open(stDatFile[0])

    # Read the data from file
    row = dataFile.readline()
    for row in dataFile.readlines():
        rowData = myParser(row)
        J.append(float(0))
        Ct.append(float(rowData[1]))
        Cq.append(float(rowData[2])/2/math.pi)

    # Close the static data file
    dataFile.close()

    # Load the dynamic data
    for i in range(len(rpmVals)):
        dataFile = open(dynDatFiles[i])  # Load the data file
        row = dataFile.readline()  # Read the data header
        for row in dataFile.readlines():
            rowData = myParser(row)
            J.append(float(rowData[0]))
            Ct.append(float(rowData[1]))
            Cq.append(float(rowData[2])/2/math.pi)

    # Fit the data to a cubic polynomial
    paramsCt, paramsCov = curve_fit(fitFun, J, Ct, bounds=(
        [-100*numpy.ones(3), 100*numpy.ones(3)]))
    paramsCq, paramsCov = curve_fit(fitFun,J, Cq, bounds=(
        [-100*numpy.ones(3), 100*numpy.ones(3)]))

    # Save the dataset
    saveFile = open("paramsCt.csv", "w")
    for param in paramsCt:
        saveFile.write(str(param) + "\n")
    saveFile.close()
    saveFile = open("paramsCq.csv", "w")
    for param in paramsCq:
        saveFile.write(str(param) + "\n")
    saveFile.close()
    saveFile = open("allData.csv", "w")
    for i in range(len(J)):
        saveFile.write(str(J[i]) + "\t" + str(Ct[i]) + "\t" + str(Cq[i]) + "\n")
    saveFile.close()


if __name__ == "__main__":
    main(sys.argv[1:])