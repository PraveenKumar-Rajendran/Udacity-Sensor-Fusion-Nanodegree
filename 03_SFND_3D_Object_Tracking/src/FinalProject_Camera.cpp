
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

// separate function for looping with different detector/descriptor combinations
int tracking_cam_lidar_ttc(string detectorType, string descriptorType, string matcherType, string descriptorType_BIN_HOG, string selectorType, bool bVis, ofstream &logfile)
{


        // print all the parameters
        cout << "detectorType: " << detectorType << endl;
        cout << "descriptorType: " << descriptorType << endl;
        cout << "matcherType: " << matcherType << endl;
        cout << "descriptorType_BIN_HOG: " << descriptorType_BIN_HOG << endl;
        cout << "selectorType: " << selectorType << endl;



    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1; 
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    // bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file 
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


        /* DETECT & CLASSIFY OBJECTS */

        float confThreshold = 0.2;
        float nmsThreshold = 0.4;
      
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                      yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);
        cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;


        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
    
        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

        cout << "#3 : CROP LIDAR POINTS done" << endl;


        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

        // Visualize 3D objects
        bVis = false;
        if(bVis)
        {
            show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(900, 450), true);
        }
        bVis = false;

        cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;
        
        
        // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
        // continue; // skips directly to the next image without processing what comes beneath

        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        // string detectorType = "SHITOMASI";

        // time the detector
        double t_det = (double)cv::getTickCount();      
      
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
      
        // time the detector
        t_det = ((double)cv::getTickCount() - t_det) / cv::getTickFrequency();

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        cout << "#5 : DETECT KEYPOINTS done" << endl;


        /* EXTRACT KEYPOINT DESCRIPTORS */

        cv::Mat descriptors;
        // string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#6 : EXTRACT DESCRIPTORS done" << endl;


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            // string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            // string descriptorType_BIN_HOG = "DES_BINARY"; // DES_BINARY, DES_HOG
            // string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType_BIN_HOG, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            
            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1)); // associate bounding boxes between current and previous frame using keypoint matches
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end()-1)->bbMatches = bbBestMatches;

            cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
            {
                // find bounding boxes associates with current match
                BoundingBox *prevBB, *currBB;
                for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                {
                    if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        currBB = &(*it2);
                    }
                }

                for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                {
                    if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        prevBB = &(*it2);
                    }
                }

                // compute TTC for current match
                if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar; 
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;
                    clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);                    
                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                    //// EOF STUDENT ASSIGNMENT

                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);
                        
                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                        string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, 4);
                        cv::imshow(windowName, visImg);
                        cout << "Press key to continue to next frame" << endl;
                        cv::waitKey(0);
                    }
                    bVis = false;

            // write the TTC values to a file        
            logfile << detectorType << "," << descriptorType << "," << matcherType << "," << descriptorType_BIN_HOG << "," << selectorType << "," << imgIndex << "," << ttcLidar << "," << ttcCamera << "," << t_det << "\n";  
                  
// Flush the output to ensure immediate writing
logfile.flush();
            // also print it to console
            cout << detectorType << "," << descriptorType << "," << matcherType << "," << descriptorType_BIN_HOG << "," << selectorType << "," << imgIndex << "," << ttcLidar << "," << ttcCamera << "," << t_det << "\n";

                } // eof TTC computation
            } // eof loop over all BB matches            

        }

    } // eof loop over all images

}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    string detectorType; // = "SHITOMASI" HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType; // = "BRISK", BRIEF, ORB, FREAK, AKAZE, SIFT
    string matcherType;        // = "MAT_BF", MAT_BF, MAT_FLANN
    string descriptorType_BIN_HOG; //  = "DES_BINARY", DES_BINARY, DES_HOG
    string selectorType;       //  = "SEL_NN", SEL_NN, SEL_KNN
    bool bVis = false;            // visualize results
  
    // write the TTC values to a file
    ofstream myfile;
    myfile.open ("../TTC_values.txt", ios::app);  
    // write the header
    myfile << "detectorType" << "," << "descriptorType" << "," << "matcherType" << "," << "descriptorType_BIN_HOG" << "," << "selectorType" << "," << "imgIndex" << "," << "ttcLidar" << "," << "ttcCamera" << "," << "t_det" << "\n";


    if (argc == 6) {
        detectorType = argv[1];
        descriptorType = argv[2];
        matcherType = argv[3];
        descriptorType_BIN_HOG = argv[4];
        selectorType = argv[5];

        tracking_cam_lidar_ttc(detectorType, descriptorType, matcherType, descriptorType_BIN_HOG, selectorType, bVis, myfile);
    }
    // check if the input is "all"
    else if (argc == 2 && strcmp(argv[1], "all") == 0)
    {

        matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
        selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
    // std::vector<std::pair<std::string, std::string>> detectorDescriptorPairs = {
    //     {"SHITOMASI", "BRISK"}, {"SHITOMASI", "BRIEF"}, {"SHITOMASI", "ORB"}, {"SHITOMASI", "FREAK"}, {"SHITOMASI", "SIFT"},
    //     {"HARRIS", "BRISK"}, {"HARRIS", "BRIEF"}, {"HARRIS", "ORB"}, {"HARRIS", "FREAK"}, {"HARRIS", "SIFT"},
    //     {"FAST", "BRISK"}, {"FAST", "BRIEF"}, {"FAST", "ORB"}, {"FAST", "FREAK"}, {"FAST", "SIFT"},
    //     {"BRISK", "BRISK"}, {"BRISK", "BRIEF"}, {"BRISK", "ORB"}, {"BRISK", "FREAK"}, {"BRISK", "SIFT"},
    //     {"ORB", "BRISK"}, {"ORB", "BRIEF"}, {"ORB", "ORB"}, {"ORB", "FREAK"}, {"ORB", "SIFT"},
    //     {"SIFT", "BRISK"}, {"SIFT", "BRIEF"}, {"SIFT", "FREAK"}, {"SIFT", "SIFT"},
    //     {"AKAZE", "BRISK"}, {"AKAZE", "BRIEF"}, {"AKAZE", "ORB"}, {"AKAZE", "FREAK"}, {"AKAZE", "SIFT"}, {"AKAZE", "AKAZE"}
    // };

    // rather declare a tuple of detector/descriptor/descriptorType_BIN_HOG
    std::vector<std::tuple<std::string, std::string, std::string>> detectorDescriptorPairs = {
        std::make_tuple("SHITOMASI", "BRISK", "DES_BINARY"), std::make_tuple("SHITOMASI", "BRIEF", "DES_BINARY"), std::make_tuple("SHITOMASI", "ORB", "DES_BINARY"), std::make_tuple("SHITOMASI", "FREAK", "DES_BINARY"), std::make_tuple("SHITOMASI", "SIFT", "DES_HOG"),
        std::make_tuple("HARRIS", "BRISK", "DES_BINARY"), std::make_tuple("HARRIS", "BRIEF", "DES_BINARY"), std::make_tuple("HARRIS", "ORB", "DES_BINARY"), std::make_tuple("HARRIS", "FREAK", "DES_BINARY"), std::make_tuple("HARRIS", "SIFT", "DES_HOG"),
        std::make_tuple("FAST", "BRISK", "DES_BINARY"), std::make_tuple("FAST", "BRIEF", "DES_BINARY"), std::make_tuple("FAST", "ORB", "DES_BINARY"), std::make_tuple("FAST", "FREAK", "DES_BINARY"), std::make_tuple("FAST", "SIFT", "DES_HOG"),
        std::make_tuple("BRISK", "BRISK", "DES_BINARY"), std::make_tuple("BRISK", "BRIEF", "DES_BINARY"), std::make_tuple("BRISK", "ORB", "DES_BINARY"), std::make_tuple("BRISK", "FREAK", "DES_BINARY"), std::make_tuple("BRISK", "SIFT", "DES_HOG"),
        std::make_tuple("ORB", "BRISK", "DES_BINARY"), std::make_tuple("ORB", "BRIEF", "DES_BINARY"), std::make_tuple("ORB", "ORB", "DES_BINARY"), std::make_tuple("ORB", "FREAK", "DES_BINARY"), std::make_tuple("ORB", "SIFT", "DES_HOG"),
        std::make_tuple("SIFT", "BRISK", "DES_BINARY"), std::make_tuple("SIFT", "BRIEF", "DES_BINARY"), std::make_tuple("SIFT", "FREAK", "DES_BINARY"), std::make_tuple("SIFT", "SIFT", "DES_HOG"),
        std::make_tuple("AKAZE", "BRISK", "DES_BINARY"), std::make_tuple("AKAZE", "BRIEF", "DES_BINARY"), std::make_tuple("AKAZE", "ORB", "DES_BINARY"), std::make_tuple("AKAZE", "FREAK", "DES_BINARY"), std::make_tuple("AKAZE", "SIFT", "DES_HOG"), std::make_tuple("AKAZE", "AKAZE", "DES_HOG")
    };

        for (auto it = detectorDescriptorPairs.begin(); it != detectorDescriptorPairs.end(); ++it)
        {
            detectorType = std::get<0>(*it);
            descriptorType = std::get<1>(*it);
            descriptorType_BIN_HOG = std::get<2>(*it);
            tracking_cam_lidar_ttc(detectorType, descriptorType, matcherType, descriptorType_BIN_HOG, selectorType, bVis, myfile);
        }
    }
     
    else 
    {
        detectorType = "SHITOMASI";
        descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
        descriptorType_BIN_HOG = "DES_BINARY"; // DES_BINARY, DES_HOG
        selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
      
        cout << "Using default values. Usage: " << argv[0] << " [detectorType] [descriptorType] [matcherType] [descriptorType_BIN_HOG] [selectorType]" << endl;
        tracking_cam_lidar_ttc(detectorType, descriptorType, matcherType, descriptorType_BIN_HOG, selectorType, bVis, myfile);
    }     


    myfile.close();

    return 0;
}
