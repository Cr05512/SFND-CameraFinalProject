#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorName, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        if(descriptorName.compare("SIFT") == 0 && normType == cv::NORM_HAMMING){
            cout << "The descriptor SIFT is not compatible with hamming distance (DES_BINARY)... falling back to L2 norm." << endl;
            normType = cv::NORM_L2;
        }

        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // Flann wants float values, so it is necessary to convert the uin8_t ones into float in order to avoid the bug
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        if(descRef.type()!=CV_32F) {
            descRef.convertTo(descRef, CV_32F);
        }

        if(descSource.type()!=CV_32F) {
            descSource.convertTo(descSource, CV_32F);
        }
        cout << "FLANN matching" << endl;
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;

        for (int i=0; i<knn_matches.size(); i++){

            if (knn_matches[i][0].distance / knn_matches[i][1].distance < minDescDistRatio)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
        cout << "Number of KeyPoints Removed = " << knn_matches.size() - matches.size() << endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, double &time)
{   
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        //int threshold = 30;        // FAST/AGAST detection threshold score.
        //int octaves = 3;           // detection octaves (use 0 to do single scale)
        //float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create();
    }
    else if (descriptorType.compare("BRIEF") == 0){

        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0){

        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0){

        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0){

        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0){

        extractor = cv::xfeatures2d::SIFT::create();
    }
    else{

        cout << "Method not found... skipping descriptor extraction." << endl;
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    time = t * 1000 / 1.0;
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, double &time, bool bVis)
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
    time = t * 1000 / 1.0;
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

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, double &time, bool bVis){
    int blockSize = 6;
    int apertureSize = 3;
    int minResponse = 50;
    double k = 0.04;
    double maxOverlap = 0.0;
    int response = 0;
    cv::KeyPoint newKeyPoint;
    bool bOverlap;
    
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    double t = (double)cv::getTickCount();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    for(size_t j = 0; j < dst_norm.rows; j++){

        for(size_t i = 0; i < dst_norm.cols; i++){

            response = (int)dst_norm.at<float>(j, i);

            if (response > minResponse)
            { 

                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        }
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    time = t * 1000 / 1.0;
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    
    if (bVis){
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorName, double &time, bool bVis){

    double t = 0;
    if(detectorName.compare("FAST") == 0){

        t = (double)cv::getTickCount();
        cv::FAST(img, keypoints, true);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "FAST with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    
    }
    else if (detectorName.compare("BRISK") == 0){
        //double briskThresh = 60;
        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> briskDetector = cv::BRISK::create();
        briskDetector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "BRISK detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    else if (detectorName.compare("ORB") == 0){

        //uint maxFeatures = 400;
        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> orbDetector = cv::ORB::create();
        orbDetector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "ORB detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        
    }
    else if (detectorName.compare("AKAZE") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> kazeDetector = cv::AKAZE::create();
        kazeDetector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "AKAZE detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        
    }
    else if (detectorName.compare("SIFT") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> siftDetector = cv::xfeatures2d::SIFT::create();
        siftDetector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
        
    }
    else{
        cout << "Non existent method selected, skipping detection..." << endl;
    }
    time = t * 1000 / 1.0;
}