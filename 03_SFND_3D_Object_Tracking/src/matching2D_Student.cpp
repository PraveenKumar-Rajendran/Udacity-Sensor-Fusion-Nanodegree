
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {

        if (descSource.type() != CV_8U || descRef.type() != CV_8U)
        {
            descSource.convertTo(descSource, CV_8U);
            descRef.convertTo(descRef, CV_8U);
        }

        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);

        cout << "BF matching cross-check=" << crossCheck;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        cout << "FLANN matching";
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;        
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knn_matches;
        double t = (double)cv::getTickCount();
      

      
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8f;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;

    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    // BRIEF, ORB, FREAK, AKAZE, SIFT
    // Reference1: https://docs.opencv.org/3.4/d7/d7a/group__xfeatures2d__experiment.html
    // Reference2: https://docs.opencv.org/3.4/d5/d51/group__features2d__main.html
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        throw invalid_argument("Invalid descriptor type: " + descriptorType);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
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

/**
 * Visualizes the keypoints detected in an image.
 * 
 * @param windowTitle Title of the window for display.
 * @param originalImage The original image on which keypoints detection was performed.
 * @param detectedKeypoints The keypoints detected in the originalImage.
 * @param visualizationImage The image used for displaying the keypoints.
 */
void visualizeKeypoints(const std::string &windowTitle, const cv::Mat &originalImage, 
                        const std::vector<cv::KeyPoint> &detectedKeypoints, cv::Mat &visualizationImage)
{
    // Draw the keypoints on the image. DRAW_RICH_KEYPOINTS flag ensures that the size and orientation of the keypoints are visualized.
    cv::drawKeypoints(originalImage, detectedKeypoints, visualizationImage, 
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Create a window with the provided title to display the image.
    cv::namedWindow(windowTitle, cv::WINDOW_AUTOSIZE);

    // Display the image with the keypoints in the created window.
    cv::imshow(windowTitle, visualizationImage);

    // Wait indefinitely until a key is pressed. This allows the user to view the image until they choose to close it.
    cv::waitKey(0);
}


// Detect keypoints in image using the traditional Harris detector
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Harris detector parameters
    int neighborhoodSize = 2;    // Size of neighborhood considered for corner detection
    int sobelApertureSize = 3;   // Aperture size for Sobel operator (must be odd)
    int minCornerStrength = 100; // Minimum value for a corner in the scaled response matrix
    double harrisFreeParam = 0.04; // Harris detector free parameter

    // Detect Harris corners and normalize output
    double t = (double)cv::getTickCount();
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, neighborhoodSize, sobelApertureSize, harrisFreeParam, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // Parameters for non-maximum suppression
    double maxOverlap = 0.0; // Max permissible overlap between two features in %
    // Reference: https://github.com/udacity/SFND_Camera/blob/master/Lesson%204%20-%20Tracking%20Image%20Features/Harris%20Corner%20Detection/solution/cornerness_harris.cpp
    // Locate local maxima in the Harris response matrix and perform NMS
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minCornerStrength)
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * sobelApertureSize;
                newKeyPoint.response = response;

                // Perform non-maximum suppression in the local neighborhood
                bool isOverlap = false;
                for (auto &existingKeyPoint : keypoints) // little optimization: use reference to avoid copying
                {
                    double overlap = cv::KeyPoint::overlap(newKeyPoint, existingKeyPoint);
                    if (overlap > maxOverlap)
                    {
                        isOverlap = true;
                        if (newKeyPoint.response > existingKeyPoint.response)
                        {
                            existingKeyPoint = newKeyPoint; // Replace with the keypoint having higher response
                            break;
                        }
                    }
                }
                if (!isOverlap)
                {
                    keypoints.push_back(newKeyPoint); // Add new keypoint if no overlap
                }
            }
        }
    }

    // Timing and output
    double timeTaken = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "HARRIS detected " << keypoints.size() << " keypoints with applied nms in " << 1000 * timeTaken << " ms" << endl;

    // Visualization
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        string windowName = "Harris Corner Detector Response Matrix";
        visualizeKeypoints(windowName, img, keypoints, visImage);
    }
}

// Detect keypoints in image using the Modern detector
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    // Reference: https://docs.opencv.org/3.4/d5/d51/group__features2d__main.html
    // Initialize detector
    cv::Ptr<cv::FeatureDetector> detector;
    if (detectorType.compare("FAST") == 0)
    {
        // FAST detector parameters
        
        int threshold = 30; // Difference between intensity of the central pixel and pixels of a circle around this pixel
        bool bNMS = true;   // Perform non-maxima suppression on keypoints
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        // BRISK detector parameters
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // Detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
        detector = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (detectorType.compare("ORB") == 0)
    {
        // ORB detector parameters
        int nFeatures = 500; // The maximum number of features to retain
        float scaleFactor = 1.2f; // Pyramid decimation ratio, greater than 1
        int nLevels = 8; // The number of pyramid levels
        int edgeThreshold = 31; // This is size of the border where the features are not detected
        int firstLevel = 0; // The level of pyramid to put source image to
        int WTA_K = 2; // The number of points that produce each element of the oriented BRIEF descriptor
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE; // The default HARRIS_SCORE means that Harris algorithm is used to rank features
        int patchSize = 31; // Size of the patch used by the oriented BRIEF descriptor
        int fastThreshold = 20; // The fast threshold
        detector = cv::ORB::create(nFeatures, scaleFactor, nLevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        // AKAZE detector parameters
        cv::AKAZE::DescriptorType descriptorType = cv::AKAZE::DESCRIPTOR_MLDB; // Type of the extracted descriptor: DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT, DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT
        int descriptorSize = 0; // Size of the descriptor in bits. 0 -> Full size
        int descriptorChannels = 3; // Number of channels in the descriptor (1, 2, 3)
        float threshold = 0.001f; // Detector response threshold to accept point
        int nOctaves = 4; // Maximum octave evolution of the image
        int nOctaveLayers = 4; // Default number of sublevels per scale level
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2; // Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER
        detector = cv::AKAZE::create(descriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers, diffusivity);
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        // SIFT detector parameters
        int nFeatures = 0; // The number of best features to retain
        int nOctaveLayers = 3; // The number of layers in each octave. 3 is the value used in D. Lowe paper
        double contrastThreshold = 0.04; // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions
        double edgeThreshold = 10; // The threshold used to filter out edge-like features
        double sigma = 1.6; // The sigma of the Gaussian applied to the input image at the octave #0
        detector = cv::xfeatures2d::SIFT::create(nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    }
    else
    {
        throw invalid_argument("Invalid detector type: " + detectorType);
    }

    // Detect keypoints
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detector detected " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // Visualization
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        string windowName = detectorType + " Detector Results";
        visualizeKeypoints(windowName, img, keypoints, visImage);
    }
}