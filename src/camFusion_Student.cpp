
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{

    for(auto itMatch = kptMatches.begin(); itMatch != kptMatches.end(); ++itMatch){
                 
        if(boundingBox.roi.contains(kptsCurr[itMatch->trainIdx].pt)){

            boundingBox.kptMatches.push_back(*itMatch);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    {

        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {

            double minDist = 100.0;

            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { 

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } 
    }     

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    float dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1/frameRate;        // time between two measurements in seconds
    double laneWidth = 2.0; // assumed width of the ego lane
    std::vector<LidarPoint> prevFiltered = RansacPlane(lidarPointsPrev,400,0.22);
    std::vector<LidarPoint> currFiltered = RansacPlane(lidarPointsCurr,400,0.22);


    // I use ransac in order to fit the car tail plane
    //double minXPrev = 0.0, minXCurr = 0.0;
    //find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = prevFiltered.begin(); it != prevFiltered.end(); ++it)
    {
        
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
        }
    }

    for (auto it = currFiltered.begin(); it != currFiltered.end(); ++it)
    {

        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            minXCurr = minXCurr > it->x ? it->x : minXCurr;
        }
    }
    std::vector<double> xValuesPrev, xValuesCurr;
    for(int i=0; i<prevFiltered.size(); i++){
        xValuesPrev.push_back(prevFiltered[i].x);
    }
    minXPrev = *std::min_element(xValuesPrev.begin(), xValuesPrev.end());

    for(int i=0; i<currFiltered.size(); i++){
        xValuesCurr.push_back(currFiltered[i].x);
    }
    minXCurr = *std::min_element(xValuesCurr.begin(), xValuesCurr.end());

    //std::cout << minXPrev << "," << minXCurr << std::endl;

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    uint bins[currFrame.boundingBoxes.size()][prevFrame.boundingBoxes.size()];
    memset(bins, 0, sizeof(bins[0][0]) * currFrame.boundingBoxes.size() * prevFrame.boundingBoxes.size());
    uint threshold = 1;
    uint max = 0, indMax = 0;
    bool processed[prevFrame.boundingBoxes.size()];
    memset(processed, false, sizeof(bool)*prevFrame.boundingBoxes.size());

    for(auto itCurr = currFrame.boundingBoxes.begin(); itCurr!=currFrame.boundingBoxes.end(); ++itCurr){
        for(auto itPrev = prevFrame.boundingBoxes.begin(); itPrev!=prevFrame.boundingBoxes.end(); ++itPrev){
             for(auto itMatch = matches.begin(); itMatch != matches.end(); ++itMatch){
                 
                 if(itCurr->roi.contains(currFrame.keypoints[itMatch->trainIdx].pt)){

                    if(itPrev->roi.contains(prevFrame.keypoints[itMatch->queryIdx].pt)){   

                         bins[itCurr->boxID][itPrev->boxID]++;
                     }
                }
             }
        }
    }

    for(int i=0; i<currFrame.boundingBoxes.size(); i++){
        for(int j=0; j<prevFrame.boundingBoxes.size(); j++){
            if(bins[i][j]>max){
                max = bins[i][j];
                indMax = j;
            }
            //cout << bins[i][j] << " "; //Uncomment this and the endl below to print the voting structure
        }
        //cout << endl;
        if(max>threshold && !processed[indMax]){
            bbBestMatches.insert({prevFrame.boundingBoxes[indMax].boxID,currFrame.boundingBoxes[i].boxID});
            //std::cout << currFrame.boundingBoxes[i].boxID << "," << prevFrame.boundingBoxes[indMax].boxID << std::endl;
            processed[indMax] = true;
        }
        max = 0;
        indMax = 0;
    }


}

std::vector<LidarPoint> RansacPlane(std::vector<LidarPoint> lidarPoints, int maxIterations, float distanceTol){
   
    srand(time(NULL));
    std::vector<LidarPoint> inliersResult;
    std::vector<LidarPoint> tempSet;

    float coefficients[4] = {0.0,0.0,0.0,0.0};

    std::vector<double> v1, v2, v3;
    std::vector<LidarPoint> samples;

    int cloudSize = lidarPoints.size();
    // For max iterations
    float distance = 0;
    for (int i=0; i<maxIterations; i++){
    // Randomly sample subset and fit plane
        samples.push_back(lidarPoints[rand()%cloudSize]);
        samples.push_back(lidarPoints[rand()%cloudSize]);
        samples.push_back(lidarPoints[rand()%cloudSize]);
        

        v1 = {samples[1].x - samples[0].x, samples[1].y - samples[0].y, samples[1].z - samples[0].z};
        v2 = {samples[2].x - samples[0].x, samples[2].y - samples[0].y, samples[2].z - samples[0].z};
        v3 = crossProd(v1,v2);

        coefficients[0] = v3[0];
        coefficients[1] = v3[1];
        coefficients[2] = v3[2];
        coefficients[3] = -(v3[0]*samples[0].x + v3[1]*samples[0].y + v3[2]*samples[0].z);
        
        // Measure distance between every point and fitted plane
        for(int j = 0; j<cloudSize; j++){
            distance = abs(coefficients[0]*lidarPoints[j].x + coefficients[1]*lidarPoints[j].y +
                           coefficients[2]*lidarPoints[j].z + coefficients[3])/sqrt(pow(coefficients[0],2)+pow(coefficients[1],2) + pow(coefficients[2],2));
            if(distance <= distanceTol){
                tempSet.push_back(lidarPoints[j]);
            }
        }
        if(tempSet.size()>inliersResult.size()){
            inliersResult = tempSet;
        }
        tempSet.clear();
        samples.clear();

    }
    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;
}

std::vector<double> crossProd(std::vector<double> const& v1, std::vector<double> const& v2){
    std::vector<double> result = {v1[1]*v2[2] - v1[2]*v2[1],
            v1[2]*v2[0] - v1[0]*v2[2],
            v1[0]*v2[1] - v1[1]*v2[0]};
    return result;
}

