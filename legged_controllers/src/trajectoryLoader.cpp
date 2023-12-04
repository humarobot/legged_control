#include "trajectoryLoader.hpp"

Eigen::MatrixXd TrajectoryLoader::LoadMatrix(std::string fileName) {
  std::vector<double> matrixEntries;
  std::ifstream matrixDataFile(fileName);
  std::string matrixRowString;
  std::string matrixEntry;
  int matrixRowNumber = 0;

  while (getline(matrixDataFile, matrixRowString))  // here we read a row by row of matrixDataFile and store every line
                                                    // into the string variable matrixRowString
  {
    std::stringstream matrixRowStringStream(
        matrixRowString);  // convert matrixRowString that is a string to a stream variable.

    while (getline(matrixRowStringStream, matrixEntry,
                   ','))  // here we read pieces of the stream matrixRowStringStream until every comma, and store the
                          // resulting character into the matrixEntry
    {
      matrixEntries.push_back(stod(matrixEntry));  // here we convert the string to double and fill in the row vector
                                                   // storing all the matrix entries
    }
    matrixRowNumber++;  // update the column numbers
  }
  return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

Eigen::MatrixXd TrajectoryLoader::GetBaseStateTrajectory() const { return stateTrajectory_.topRows(6); }

Eigen::MatrixXd TrajectoryLoader::GetBaseVelTrajectory() const { return velTrajectory_.topRows(6); }


