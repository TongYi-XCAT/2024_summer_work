#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

array<Matrix<double, Dynamic, Dynamic>, 2> KFpredict(
    Matrix<double, Dynamic, Dynamic> Xb, Matrix<double, Dynamic, Dynamic> Ub,
    Matrix<double, Dynamic, Dynamic> Pb, Matrix<double, Dynamic, Dynamic> A,
    Matrix<double, Dynamic, Dynamic> B,  Matrix<double, Dynamic, Dynamic> Q);

array<Matrix<double, Dynamic, Dynamic>, 3> KFupdate(
    Matrix<double, Dynamic, Dynamic> Xe, Matrix<double, Dynamic, Dynamic> Pe,
    Matrix<double, Dynamic, Dynamic> Z, Matrix<double, Dynamic, Dynamic> H,
    Matrix<double, Dynamic, Dynamic> R);