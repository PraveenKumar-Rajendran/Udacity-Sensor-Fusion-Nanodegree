
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
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

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

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
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

// Function to calculate the median of a vector of doubles
double calculateMedian(const std::vector<double>& values) {
    std::vector<double> sortedValues = values;  // Make a copy to avoid modifying the original vector
    std::sort(sortedValues.begin(), sortedValues.end());

    unsigned long size = sortedValues.size();  // Use unsigned long for the size variable
    if (size % 2 == 0) {
        // Even number of elements, average the middle two
        return (sortedValues[size / 2 - 1] + sortedValues[size / 2]) / 2.0;
    } else {
        // Odd number of elements, return the middle value
        return sortedValues[size / 2];
    }
}


// Associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // Initialize variables
    float meanDistanceRatio = 1.4f;
    float threshold = 0.1f;  // Threshold for outlier removal

    // Store Euclidean distances for each match
    std::vector<double> euclideanDistances;

    // Step 1: Associate keypoints from the current frame with the bounding box
    for (const auto &match : kptMatches)
    {
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            boundingBox.kptMatches.emplace_back(match);

            // Calculate the Euclidean distance for the current match
            float euclideanDistance = sqrt(pow((kptsCurr[match.trainIdx].pt.x - kptsPrev[match.queryIdx].pt.x), 2.0) + pow((kptsCurr[match.trainIdx].pt.y - kptsPrev[match.queryIdx].pt.y), 2.0));

            // Store distances
            euclideanDistances.push_back(euclideanDistance);
        }
    }

  // CHOOSE ONE OF THE TWO!! :))
    // Calculate the median Euclidean distance
//     float distanceThreshold = calculateMedian(euclideanDistances);
    // Calculate the mean Euclidean distance
	float distanceThreshold = std::accumulate(euclideanDistances.begin(), euclideanDistances.end(), 0.0f) / euclideanDistances.size();


  
  
    // Step 2: Remove outliers based on the distance threshold and ratio
    std::vector<cv::DMatch> filteredMatches;
    for (size_t i = 0; i < boundingBox.kptMatches.size(); ++i)
    {
        float euclideanDistance = euclideanDistances[i];
        float distanceRatio = euclideanDistance / distanceThreshold;

        if (distanceRatio <= meanDistanceRatio)
        {
            // Keep the match
            filteredMatches.push_back(boundingBox.kptMatches[i]);
        }
    }

    // Replace kptMatches with filteredMatches
    boundingBox.kptMatches = filteredMatches;

    // Print a message indicating the completion of the function
    cout << "#12 : Finish clusterKptMatchesWithROI" << endl;
}



// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
// Reference: https://github.com/udacity/SFND_Camera/blob/master/Lesson%203%20-%20Engineering%20a%20Collision%20Detection%20System/Estimating%20TTC%20with%20Camera/solution/compute_ttc_camera.cpp
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // STUDENT TASK (replacement for meanDistRatio)

    double medDistRatio = calculateMedian(distRatios);

    double dT = 1 / frameRate;
  
    TTC = -dT / (1 - medDistRatio);
  cout << "#13 : Finish computeTTCCamera" << endl;

}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Calculate the time interval (dT) between two measurements in seconds
    double dT = 1 / frameRate;

    // Assumed width of the ego lane
    double laneWidth = 4.0;

    // Calculate a threshold for an "edge case" based on lane width
    double egolane_center_side_width = (laneWidth) / 2;

    // Lambda function to check if a Lidar point is within the ego lane
    // Reference: https://studiofreya.com/cpp/std-remove_if-with-lambda/

    auto check_ego_lane_points = [&egolane_center_side_width](const LidarPoint &lidar_p)
    { 
        return abs(lidar_p.y) >= (egolane_center_side_width - 0.1); 
	// 0.1 is subtracted to filter out some more points in the edge since they tend to associate to other objects.
    };

    // Remove Lidar points that are outside the ego lane in the previous and current frames
    lidarPointsPrev.erase(std::remove_if(lidarPointsPrev.begin(), lidarPointsPrev.end(), check_ego_lane_points),
                          lidarPointsPrev.end());

    lidarPointsCurr.erase(std::remove_if(lidarPointsCurr.begin(), lidarPointsCurr.end(), check_ego_lane_points),
                          lidarPointsCurr.end());

    // Create vectors to store x-coordinates of Lidar points in the previous and current frames
    vector<double> lidarPointsCurrX, lidarPointsPrevX;


// Define a lambda function to extract the x-coordinate from a LidarPoint
auto extractXCoordinate = [](const LidarPoint& point) {
    return point.x;
};


    // Populate the vectors with x-coordinates of Lidar points
// Extract x-coordinates using the common lambda function
std::transform(lidarPointsPrev.begin(), lidarPointsPrev.end(), std::back_inserter(lidarPointsPrevX), extractXCoordinate);
std::transform(lidarPointsCurr.begin(), lidarPointsCurr.end(), std::back_inserter(lidarPointsCurrX), extractXCoordinate);

    // Sort the vectors of x-coordinates to easily find the median by using mid index
    sort(lidarPointsCurrX.begin(), lidarPointsCurrX.end());
    sort(lidarPointsPrevX.begin(), lidarPointsPrevX.end());

// Calculate the median x-coordinate in the current frame
double d1 = calculateMedian(lidarPointsCurrX);

// Calculate the median x-coordinate in the previous frame
double d0 = calculateMedian(lidarPointsPrevX);

    // Compute the Time-to-Collision (TTC) using Lidar data (constant velocity model)
    TTC = d1 * dT / (d0 - d1);
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    // Create a multimap to record the associations between current and previous bounding boxes.
    std::multimap<int, int> boxAssociations;

    // Step 1: Associate keypoints with bounding boxes
    for (auto match : matches) {
        cv::KeyPoint prevKeypoint = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint currKeypoint = currFrame.keypoints[match.trainIdx];

        int prevBoxId = -1;
        int currBoxId = -1;

        // Find the previous bounding box containing the previous keypoint.
        for (const auto &prevBox : prevFrame.boundingBoxes) {
            if (prevBox.roi.contains(prevKeypoint.pt)) {
                prevBoxId = prevBox.boxID;
                break; // No need to search further.
            }
        }

        // Find the current bounding box containing the current keypoint.
        for (const auto &currBox : currFrame.boundingBoxes) {
            if (currBox.roi.contains(currKeypoint.pt)) {
                currBoxId = currBox.boxID;
                break; // No need to search further.
            }
        }

        // Record the mapping between current and previous box IDs.
        if (prevBoxId != -1 && currBoxId != -1) {
            boxAssociations.insert({currBoxId, prevBoxId});
        }
    }

    // Step 2: Determine the best match for each current bounding box
    const int numCurrentBoxes = currFrame.boundingBoxes.size();
    const int numPreviousBoxes = prevFrame.boundingBoxes.size();

    for (int currBoxIdx = 0; currBoxIdx < numCurrentBoxes; ++currBoxIdx) {
        // Get all associations for the current box.
        auto associations = boxAssociations.equal_range(currBoxIdx);

        // Count the number of associations with each previous box.
        std::vector<int> prevBoxCounts(numPreviousBoxes, 0);
        for (auto it = associations.first; it != associations.second; ++it) {
            int prevBoxId = it->second;
            if (prevBoxId >= 0 && prevBoxId < numPreviousBoxes) {
                prevBoxCounts[prevBoxId]++;
            }
        }

        // Find the previous box with the most associations.
        int bestPrevBoxIdx = std::distance(prevBoxCounts.begin(),
                                           std::max_element(prevBoxCounts.begin(), prevBoxCounts.end()));

        // Store the best match in the output map.
        bbBestMatches[currBoxIdx] = bestPrevBoxIdx;

        // Debugging output (can be removed in production code).
        std::cout << "Current Box ID: " << currBoxIdx << " matches Previous Box ID: " << bestPrevBoxIdx << std::endl;
    }
}