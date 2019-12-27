#include <iostream>
#include <chrono>
#include "find_matches.h"
#include <glog/logging.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }
    Mat image1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat image2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    if (image1.data == nullptr || image2.data == nullptr)
        LOG(FATAL) << "invalid file";
    vector<DMatch> good_matches;
    // find_matches(image1, image2,good_matches );
    return 0;
}
