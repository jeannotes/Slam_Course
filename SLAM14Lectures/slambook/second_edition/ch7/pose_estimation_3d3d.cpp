#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <glog/logging.h>

using namespace std;
using namespace cv;

void find_feature_matches(
    const Mat &img_1, const Mat &img_2,
    std::vector<KeyPoint> &keypoints_1,
    std::vector<KeyPoint> &keypoints_2,
    std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t);

void bundleAdjustment(
    const vector<Point3f> &points_3d,
    const vector<Point3f> &points_2d,
    Mat &R, Mat &t);

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d point) : _point(point)
    {
    }
    virtual void computeError()
    {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        _error = _measurement - pose->estimate().map(_point);
    }

    virtual void linearizeOplus()
    {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d error = pose->estimate().map(_point);
        double x = error(0, 0);
        double y = error(1, 0);
        double z = error(2, 0);
        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}

protected:
    Eigen::Vector3d _point;
};

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cout << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat depth1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED); // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED); // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts1, pts2;

    for (DMatch m : matches)
    {
        ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
        ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
        if (d1 == 0 || d2 == 0) // bad depth
            continue;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
        float dd1 = float(d1) / 5000.0;
        float dd2 = float(d2) / 5000.0;
        pts1.push_back(Point3f(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push_back(Point3f(p2.x * dd2, p2.y * dd2, dd2));
    }

    cout << "3d-3d pairs: " << pts1.size() << endl;
    Mat R, t;
    pose_estimation_3d3d(pts1, pts2, R, t);
    cout << "ICP via SVD results: " << endl;
    cout << "R = " << R << endl;
    cout << "t = " << t << endl;
    cout << "R_inv = " << R.t() << endl;
    cout << "t_inv = " << -R.t() * t << endl;

    cout << "calling bundle adjustment" << endl;

    bundleAdjustment(pts1, pts2, R, t);

    // // verify p1 = R*p2 + t
    // for ( int i=0; i<5; i++ )
    // {
    //     cout<<"p1 = "<<pts1[i]<<endl;
    //     cout<<"p2 = "<<pts2[i]<<endl;
    //     cout<<"(R*p2+t) = "<<
    //         R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
    //         <<endl;
    //     cout<<endl;
    // }
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches)
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match(descriptors_1, descriptors_2, match);

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = match[i].distance;
        if (dist < min_dist)
            min_dist = dist;
        if (dist > max_dist)
            max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (match[i].distance <= max(2 * min_dist, 30.0))
        {
            matches.push_back(match[i]);
        }
    }
}

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
    return Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void pose_estimation_3d3d(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    if (pts1.empty() || pts2.empty())
        LOG(ERROR) << "ERROR";
    Eigen::Vector3d p1(0, 0, 0), p2(0, 0, 0);
    for (int i = 0; i < pts1.size(); i++)
    {
        p1 += Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z);
        p2 += Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z);
    }
    p1 = p1 / pts1.size();
    p2 = p2 / pts1.size();
    // subtract mean value
    vector<Eigen::Vector3d> pts1_subtract, pts2_subtract;
    // compute W
    for (int i = 0; i < pts1.size(); i++)
    {
        Eigen::Vector3d p = Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z) - p1;
        pts1_subtract.push_back(p);
        p = Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) - p2;
        pts2_subtract.push_back(p);
    }
    Eigen::Matrix3d W;
    for (int i = 0; i < pts1.size(); i++)
    {
        W += pts1_subtract[i] * pts2_subtract[i].transpose();
    }
    LOG(WARNING) << "COL: " << W.cols() << " rows: " << W.rows();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d u = svd.matrixU();
    Eigen::Matrix3d v = svd.matrixV();
    Eigen::Vector3d sigma = svd.singularValues();
    auto tt = (sigma * sigma.transpose()).array();
    if (u.determinant() * v.determinant() < 0)
    {
        for (int j = 0; j < 3; ++j)
        {
            u(j, 2) = -u(j, 2);
        }
    }
    Eigen::Matrix3d R_mat = u * (v.transpose());
    Eigen::Vector3d t_mat = p1 - R_mat * p2;
    R = (Mat_<double>(3, 3) << R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
         R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
         R_mat(2, 0), R_mat(2, 1), R_mat(2, 2));
    t = (Mat_<double>(3, 1) << t_mat(0, 0), t_mat(1, 0), t_mat(2, 0));
}

void bundleAdjustment(
    const vector<Point3f> &pts1,
    const vector<Point3f> &pts2,
    Mat &R, Mat &t)
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    //add vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(pose);
    //add edge
    int index = 1;
    for (int i = 0; i < pts1.size(); i++)
    {
        EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
        edge->setId(index);
        edge->setInformation(Eigen::Matrix3d::Identity() * 1e4);
        edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap *>(pose));
        optimizer.addEdge(edge);
        index++;
    }
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout << Eigen::Isometry3d(pose->estimate()).matrix();
}
/*
3d-3d pairs: 75
W=  11.8688 -0.717698   1.89486
 -1.88065   3.83391  -5.78219
  3.25846  -5.82734   9.65267
U=  0.592295  -0.805658 -0.0101195
 -0.418171  -0.318113   0.850845
  0.688709   0.499719   0.525319
V=   0.64736  -0.761401 -0.0345329
 -0.388765  -0.368829   0.844291
  0.655581   0.533135   0.534772
ICP via SVD results: 
R = [0.9972065647956201, 0.05834273442898391, -0.04663895869192625;
 -0.05787745545449197, 0.998260122172866, 0.01126626067193237;
 0.04721511705620757, -0.008535444848613793, 0.9988482762174666]
t = [0.1379879629890433;
 -0.06551699470729988;
 -0.02981649388290575]
R_inv = [0.9972065647956201, -0.05787745545449197, 0.04721511705620757;
 0.05834273442898391, 0.998260122172866, -0.008535444848613793;
 -0.04663895869192625, 0.01126626067193237, 0.9988482762174666]
t_inv = [-0.1399866702492459;
 0.05709791102272541;
 0.03695589996443214]
calling bundle adjustment


*/