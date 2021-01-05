#include <numeric>
#include "matching2D.hpp"

using namespace std;

void wipeNeighbors(cv::Mat &mat, const cv::Point &center, int aspect)
{
    int aspect_multiple = 1;
    for (int i = -aspect * aspect_multiple; i <= aspect * aspect_multiple; i++)
    {
        for (int j = -aspect * aspect_multiple; j <= aspect * aspect_multiple; j++)
        {
            if ((i == 0) && (j == 0))
                continue;
            if (center.x - i < 0)
                continue;
            if (center.x + i >= mat.cols)
                continue;
            if (center.y - j < 0)
                continue;
            if (center.y + j >= mat.rows)
                continue;
            //       cout << "Seetting to 0 " << i << " " << j << endl;
            mat.at<unsigned char>(center.y + j, center.x + i) = 0;
        }
    }
    mat.at<unsigned char>(center.y, center.x) = 255;
}
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    // cout << "meow img" << endl;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(dst_norm_scaled, mean, stddev);
    unsigned char harris_mean = mean.val[0];
    unsigned char harris_sttdev = stddev.val[0];
    // Create new matrix
    // cout << "Mean is " << int(harris_mean) << " sttdev is " << int(harris_sttdev) << "" << endl;
    cv::Mat keypoint_candidates = dst_norm_scaled - 1.5 * harris_mean;

    // For all points > 0
    // Find the biggest point, set all points in a 2x aperture window to 0, then move to next point, if next point was cleared, continue
    // compute magnitude image
    // Sort through sorted array from
    // https://stackoverflow.com/questions/17975402/iterate-through-cvpoints-contained-in-a-cvmat
    cv::Mat_<int> indices;
    cv::sortIdx(keypoint_candidates.reshape(0, 1), indices, cv::SORT_DESCENDING + cv::SORT_EVERY_ROW);
    assert(keypoint_candidates.at<float>(1, 0) == keypoint_candidates.reshape(0, 1).at<float>(0, 375));
    int points_counted = 0;
    int col_size = keypoint_candidates.cols;
    for (int i = 0; i < indices.cols; i++)
    {
        int idx = indices.at<int>(0, i);
        int row_num = int(idx / col_size);
        int col_num = idx % col_size;
        unsigned char val = keypoint_candidates.at<unsigned char>(row_num, col_num);
        if (val < 2)
            continue;
        points_counted++;
        keypoints.push_back(cv::KeyPoint(col_num, row_num, blockSize));
        cv::Point point(col_num, row_num);
        wipeNeighbors(keypoint_candidates, point, apertureSize);
        // Do something with pt...
    }
    //   cout << " Total points is " << dst_norm_scaled.rows * dst_norm_scaled.cols << endl;
    //   cout << " Points counted is is " << points_counted << endl;
    //   cout << " Total kp is " << keypoints.size() << "" << endl;
}
// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = true;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // ...
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::FlannBasedMatcher::create();

        //... TODO : implement FLANN matching
        cout << "FLANN matching";
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector<std::vector<cv::DMatch>> knnmatches;
        matcher->knnMatch(descRef, descSource, knnmatches, 2);
        // TODO : filter matches using descriptor distance ratio test
        for (auto match : knnmatches)
        {
            if (match.at(0).distance / match.at(1).distance >= 0.8)
                continue;
            matches.push_back(match.at(0));
        }
        cout << "meow " << matches.size() << endl;
        // ...
    }
}

constexpr unsigned int str2int(const char *str, int h = 0)
{
    // From https://stackoverflow.com/questions/16388510/evaluate-a-string-with-a-switch-in-c/16388594
    return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    switch (str2int(descriptorType.c_str()))
    {
    case str2int("BRISK"):
        extractor = cv::BRISK::create(30, 3, 1.0f);
        break;
    case str2int("BRIEF"):
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
        break;
    case str2int("ORB"):
        extractor = cv::ORB::create();
        break;
    case str2int("FREAK"):
        extractor = cv::xfeatures2d::FREAK::create();
        break;
    case str2int("AKAZE"):
        extractor = cv::AKAZE::create();
        break;
    case str2int("SIFT"):
        // extractor = cv::xfeatures2d::SIFT::create();
        extractor = cv::SIFT::create();
        break;
    default:
        break;
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const std::string detectorType, bool bVis)
{
    cv::Ptr<cv::Feature2D> detector;
    switch (str2int(detectorType.c_str()))
    {
    case str2int("FAST"):
        /* code */
        detector = cv::FastFeatureDetector::create();
        break;
    case str2int("BRISK"):
        /* code */
        detector = cv::BRISK::create();
        break;
    case str2int("ORB"):
        /* code */
        detector = cv::ORB::create();
        break;
    case str2int("AKAZE"):
        /* code */
        detector = cv::AKAZE::create();
        break;
    case str2int("SIFT"):
        /* code */
        // detector = cv::xfeatures2d::SIFT::create();
        detector = cv::SIFT::create();
        break;
    default:
        break;
    }
    detector->detect(img, keypoints);
    return;
}