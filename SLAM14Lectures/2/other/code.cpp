#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;

int main(){
    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(0, 1) = 2;
    m(1, 0) = 1;
    m(1, 1) = 0;
    std::cout << m <<std::endl;
}
