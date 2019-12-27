#include "./find_matches.h"

using namespace std;
using namespace cv;

int find_matches(const Mat img_1, const Mat img_2,
                 vector<KeyPoint> &keypoints1,
                 vector<KeyPoint> &keypoints2,
                 vector<DMatch> &good_matches)
{
    //detect
    Ptr<ORB> detector = ORB::create();
    detector->detect(img_1, keypoints1);
    detector->detect(img_2, keypoints2);
    //debug - draw points
    // drawKeypoints(img_1, keypoints1, img_1);
    // imshow("debug", img_1);
    // waitKey(0);
    //descriptor
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Mat descriptors1, descriptors2;
    descriptor->compute(img_1, keypoints1, descriptors1);
    descriptor->compute(img_2, keypoints2, descriptors2);
    // matcher
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);
    // Mat all_matches_img;
    // drawMatches(img_1, keypoints1, img_2, keypoints2, matches, all_matches_img);
    // imshow("all-matches", all_matches_img);
    // waitKey(0);
    // get min&max distance
    auto min_max = std::minmax_element(matches.begin(), matches.end(),
                                       [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < std::max(30.0, double(2 * min_max.first->distance)))
        {
            good_matches.push_back(matches[i]);
        }
    }
    //now we have good matches
    // Mat all_matches_img;
    // drawMatches(img_1, keypoints1, img_2, keypoints2, good_matches, all_matches_img);
    // imshow("all-matches", all_matches_img);
    // waitKey(0);
}

int find_corresponding_points(const Mat depth1,
                              const Mat depth2,
                              const Mat k,
                              vector<KeyPoint> keypoints1,
                              vector<KeyPoint> keypoints2,
                              vector<DMatch> &good_matches,
                              vector<Point3f> &points_3d,
                              vector<Point2f> &points_2d)
{
    if (good_matches.empty())
        return 0;
    auto pixel_to_3d_point = [](Point2f p, Mat k, unsigned short depth) {
        double x = (p.x - k.at<double>(0, 2)) / k.at<double>(0, 0);
        double y = (p.y - k.at<double>(1, 2)) / k.at<double>(1, 2);
        depth = depth / 5000.0;
        return Point3f(x * depth, y * depth, depth);
    };
    LOG_IF(WARNING, 0) << "OK";
    for (const DMatch match : good_matches)
    {
        //choose 3d point from image1, which is query point
        int y1 = keypoints1[match.queryIdx].pt.y, x1 = keypoints1[match.queryIdx].pt.x;
        unsigned short d1 = depth1.ptr<unsigned short>(y1)[x1];
        if (d1 == 0.)
            continue;
        //choose 2d point from image2, which is train point
        int y2 = keypoints2[match.trainIdx].pt.y;
        int x2 = keypoints2[match.trainIdx].pt.x;
        unsigned short d2 = depth2.ptr<unsigned short>(y2)[x2];
        // if (d2 == 0)
        //     continue;
        Point3f image1_3d_point = pixel_to_3d_point(keypoints1[match.queryIdx].pt, k, d1);
        Point2f image2_2d_point = keypoints2[match.trainIdx].pt;
        // now push into vectors
        points_3d.push_back(image1_3d_point);
        points_2d.push_back(image2_2d_point);
    }
}