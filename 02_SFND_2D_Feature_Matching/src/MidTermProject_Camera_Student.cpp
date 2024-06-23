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

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
	// set random seed
//   	cv::theRNG().state = time(NULL);

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // Find average time for each detector and descriptor in two variables
    double total_time_detector = 0.0;
    double total_time_descriptor = 0.0;
    double total_time_matcher = 0.0;

    // find total number of keypoints for all images and matches for all images
    double total_keypoints = 0;
    double total_matches = 0;
  
    // Controlling parameters for the project
    string detectorType; // = "SHITOMASI" HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptorType; // = "BRISK", BRIEF, ORB, FREAK, AKAZE, SIFT
    string matcherType;        // = "MAT_BF", MAT_BF, MAT_FLANN
    string descriptorType_BIN_HOG; //  = "DES_BINARY", DES_BINARY, DES_HOG
    string selectorType;       //  = "SEL_NN", SEL_NN, SEL_KNN
      
    if (argc == 6) {
        detectorType = argv[1];
        descriptorType = argv[2];
        matcherType = argv[3];
        descriptorType_BIN_HOG = argv[4];
        selectorType = argv[5];
    } else {
        cout << "Using default values. Usage: " << argv[0] << " [detectorType] [descriptorType] [matcherType] [descriptorType_BIN_HOG] [selectorType]" << endl;
    }     
  
        // print all the parameters
        cout << "detectorType: " << detectorType << endl;
        cout << "descriptorType: " << descriptorType << endl;
        cout << "matcherType: " << matcherType << endl;
        cout << "descriptorType_BIN_HOG: " << descriptorType_BIN_HOG << endl;
        cout << "selectorType: " << selectorType << endl;
    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
//         dataBuffer.push_back(frame);
        if (dataBuffer.size() == dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
            cout<<"Buffer is full, removing the first element"<<endl;
        }
        // add the new frame to the end of the buffer
        cout<<"Adding new frame to the buffer"<<endl;
        dataBuffer.push_back(frame);
      
      
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
      
      	// Controlling parameters for the project
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        cv::Mat descriptors;
// 		vector<cv::DMatch> matches;
      
      

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
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

        // add the time to the total time
        total_time_detector += t_det;
      
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // cout<<"Removing keypoints outside the vehicle rectangle"<<endl;
            vector<cv::KeyPoint> keypointsInVehicleRect;
            for (auto keypoint : keypoints)
            {
                if (vehicleRect.contains(keypoint.pt))
                {
                    keypointsInVehicleRect.push_back(keypoint);
                }
            }
            keypoints = keypointsInVehicleRect;
            cout<<"Number of keypoints in the defined vehicle rectangle: "<<keypoints.size()<<endl;
        }
        else
        {
            cout<<"Number of keypoints: "<<keypoints.size()<<endl;
        }

        // add the number of keypoints to the total number of keypoints
        total_keypoints += keypoints.size();
      
        //// EOF STUDENT ASSIGNMENT

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
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        // time the descriptor
        double t_desc = (double)cv::getTickCount();

        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
     
		t_desc = ((double)cv::getTickCount() - t_desc) / cv::getTickFrequency();

        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;



        // add the time to the total time
        total_time_descriptor += t_desc;
      
      
        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
//             string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
//             string descriptorType_BIN_HOG = "DES_BINARY"; // DES_BINARY, DES_HOG
//             string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
          
          
            // time the matcher
            double t_match = (double)cv::getTickCount();

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType_BIN_HOG, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            // time the matcher
            t_match = ((double)cv::getTickCount() - t_match) / cv::getTickFrequency();

            // add the time to the total time
            total_time_matcher += t_match;
          
            // add the number of matches to the total number of matches
            total_matches += matches.size();
          
            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    // print out a seperator so that the output is easier to read

    cout << "--------------------------------------------------" << endl;
    
    // print the average time for each detector and descriptor, display in ms (1000 * t / 1.0)
    cout << "Average time for detector: " << ((1000*(total_time_detector / (imgEndIndex + 1))) / 1.0) << " ms" << endl;
//     cout << "Average time for descriptor: " << ((1000*(total_time_descriptor / (imgEndIndex + 1))) / 1.0) << " ms" << endl;
    cout << "Average time for matcher: " << ((1000*(total_time_matcher / imgEndIndex)) / 1.0) << " ms" << endl;

    // print average number of keypoints
    cout << "Average number of keypoints: " << total_keypoints / (imgEndIndex + 1) << endl;
    cout << "Average number of matches: " << total_matches / imgEndIndex << endl;
//   	cout << "OpenCV: "<< cv::getBuildInformation().c_str() << endl;
  
    return 0;
}
