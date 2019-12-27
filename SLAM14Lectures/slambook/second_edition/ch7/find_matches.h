#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <glog/logging.h>
using namespace std;
using namespace cv;
int find_matches(Mat img_1, Mat img_2,
                 vector<KeyPoint> &keypoints1,
                 vector<KeyPoint> &keypoints2,
                 vector<DMatch> &good_matches);
                 
int find_corresponding_points(const Mat depth1,
                              const Mat depth2,
                              const Mat k,
                              vector<KeyPoint> keypoints1,
                              vector<KeyPoint> keypoints2,
                              vector<DMatch> &good_matches,
                              vector<Point3f> &points_3d,
                              vector<Point2f> &points_2d);