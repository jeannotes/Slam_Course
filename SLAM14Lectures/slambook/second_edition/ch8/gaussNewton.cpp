#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>

using namespace std;
using namespace Eigen;
using namespace cv;

int main(int argc, char **argv)
{
    LOG(WARNING) << "DDDDD";
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = 3.0, ce = 4.0;
    vector<double> x_data, y_data;
    double w_sigma = 1.;
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;
    for (int i = 0; i < 100; i++)
    {
        double x = i / 100;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    double cost = 0, lastCost = 0;
    for (int iter = 0; iter < 100; iter++)
    {
        cost = 0;
        Eigen::Vector3d J;
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        LOG(WARNING) << x_data.size();
        for (int i = 0; i < x_data.size(); i++)
        {
            double error = y_data[i] - exp(ae * x_data[i] * x_data[i] + be * x_data[i] + ce);
            double temp = exp(ae * x_data[i] * x_data[i] + be * x_data[i] + ce);
            J(0, 0) = -temp * x_data[i] * x_data[i];
            J(1, 0) = -temp * x_data[i];
            J(2, 0) = -temp;

            H += J * J.transpose();
            b += -error * J;
            cost += error * error;
        }
        // 求解线性方程 Hx=b
        Vector3d dx = H.ldlt().solve(b);
        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)
        {
            cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
            break;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];
        lastCost = cost;
        cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    }
    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;
    return 0;
}