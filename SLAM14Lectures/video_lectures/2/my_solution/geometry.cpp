#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace std;

int main( int argc, char** argv )
{
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3d t1(0.3, 0.1, 0.1);
    Eigen::Vector3d t2(-0.1, 0.5, 0.3);
    Eigen::Vector3d p1(0.5, 0, 0.2);
    
    Eigen::Quaterniond q1_one = q1.normalized();
    Eigen::Quaterniond q2_one = q2.normalized();
    
    
    //way1
    
    Eigen::Vector3d v = q1_one.inverse() * (p1 - t1);
    Eigen::Vector3d v2 = q2_one * v + t2;
    cout << "way1 v2 = " << endl << v2 << endl;
    return 0;
}