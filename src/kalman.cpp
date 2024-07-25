#include <../inc/kalman.hpp>

array<Matrix<double, Dynamic, Dynamic>, 2> KFpredict(
    Matrix<double, Dynamic, Dynamic> Xb, Matrix<double, Dynamic, Dynamic> Ub,
    Matrix<double, Dynamic, Dynamic> Pb, Matrix<double, Dynamic, Dynamic> A,
    Matrix<double, Dynamic, Dynamic> B,  Matrix<double, Dynamic, Dynamic> Q)
{
    Matrix<double, Dynamic, Dynamic> Xe = A * Xb + B * Ub;
    Matrix<double, Dynamic, Dynamic> Pe = A * Pb * A.transpose() + Q;
    array<Matrix<double, Dynamic, Dynamic>, 2> temp;
    temp[0] = Xe;
    temp[1] = Pe;
    return temp;
}

array<Matrix<double, Dynamic, Dynamic>, 3> KFupdate(
    Matrix<double, Dynamic, Dynamic> Xe, Matrix<double, Dynamic, Dynamic> Pe,
    Matrix<double, Dynamic, Dynamic> Z, Matrix<double, Dynamic, Dynamic> H,
    Matrix<double, Dynamic, Dynamic> R)
{
    Matrix<double, Dynamic, Dynamic> K = Pe * H.transpose() * (H * Pe * H.transpose() + R).inverse();
    Matrix<double, Dynamic, Dynamic> X = Xe + K * (Z - H * Xe);
    MatrixXd I = MatrixXd::Identity(K.rows(), K.rows());
    Matrix<double, Dynamic, Dynamic> P = (I - K * H) * Pe;
    array<Matrix<double, Dynamic, Dynamic>, 3> temp;
    temp[0] = K;
    temp[1] = X;
    temp[2] = P;
    return temp;
}