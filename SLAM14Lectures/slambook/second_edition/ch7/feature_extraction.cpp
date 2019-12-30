#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "./find_matches.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    if (argc != 3)
    {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }

    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    vector<KeyPoint> keypoints1, keypoints2;
    vector<DMatch> good_matches;
    find_matches(img_1, img_2,
                 keypoints1,
                 keypoints2,
                 good_matches);

    Mat img_match;
    Mat img_goodmatch;
    // drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints1, img_2, keypoints2, good_matches, img_goodmatch);
    // imshow("all pairs", img_match);
    imshow("after optimize all pairs", img_goodmatch);
    waitKey(0);

    return 0;
}