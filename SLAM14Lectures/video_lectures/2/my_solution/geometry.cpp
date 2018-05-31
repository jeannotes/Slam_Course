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
    /** first translation
     * first translation, qi_one is world to camera
     * (p1 - t1) is in camera coordinate, after that , it is in world coordinator
     * so , we can use the same way to compute another point
     */ 

    return 0;
}