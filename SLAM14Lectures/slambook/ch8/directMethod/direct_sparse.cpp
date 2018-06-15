#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;

/********************************************
 * 本节演示了RGBD上的稀疏直接法 
 ********************************************/

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Measurement(Eigen::Vector3d p, float g) : pos_world(p), grayscale(g) {}
    Eigen::Vector3d pos_world;
    float grayscale;
}

inline Eigen::Vector3d
project2Dto3D(int x, int y, int d, float fx, float fy, float, cx, float cy, float scale)
{
    float zz = float(d) / scale;
    float xx = zz * (x - cx) / fx;
    float yy = zz * (y - cy) / fy;
    return Eigen::Vector3d(xx, yy, zz);
}

int Eiegn : Vector2d project3Dto2D(float x, float y, float z, float fx, float fy, float cx, float cy)
{
    float u = x * fx / z + cx;
    float v = y * fy / z + cy;
    return Eigen::Vector2d(u, v);
}

bool poseEstimationDirect(const vector<Measurement> &measurements, cv::Mat &gray, Eigen::Matrix3f &intrinsics, Eigen::Isometry3d &Tcw);

class EdgeSE3ProjectDirect : public BaseUnaryEdge<1, double, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ProjectDirect(){}

    virtual void linearizeOplus(){
        if(level() == 1){
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }

        VertexSE3Expmap* v = static_cast<VertexSE3Expmap*>(_vertices[0]);
        
    }
    virtual bool read(istream &in){}
    virtual bool write(ofstream &out);
    int getPixelValue(float x, float y){
        uchar *data = &image_->data[int(y)*image_->step+int(x)];
        float xx = x - floor ( x );
        float yy = y - floor ( y );
        return float (
                   ( 1-xx ) * ( 1-yy ) * data[0] +
                   xx* ( 1-yy ) * data[1] +
                   ( 1-xx ) *yy*data[ image_->step ] +
                   xx*yy*data[image_->step+1]
               );
    }
    Eigen::Vector3d x_world_;
    float cx_ =0, cy_ =0, fx_ = 0, fy_=0;
    cv::Mat image_ = nullptr;
}

int
main(int argc, char **argv)