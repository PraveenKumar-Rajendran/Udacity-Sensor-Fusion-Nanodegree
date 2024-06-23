# SFND Project 2 - Mid-Term Report

This README serves as a mid-term report for the Sensor Fusion Nanodegree (SFND) Project 2, addressing the rubric points in order.

## Data Buffer
### MP.1 Data Buffer Optimization

A vector for dataBuffer objects with a maximum size of 2 elements is implemented. This is achieved by pushing new elements on one end and removing elements on the other end, as seen in lines 96-107 of `MidTermProject_Camera_Student.cpp`.

## Keypoints
### MP.2 Keypoint Detection

HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT detectors are implemented using OpenCV methods. Function definitions can be found in the `matching2D_Student.cpp` file.

### MP.3 Keypoint Removal

All keypoints outside of a predefined rectangle are removed, leaving only the keypoints within the rectangle for further processing. The implementation can be found in lines 155-174 of `MidTermProject_Camera_Student.cpp`.

## Descriptors
### MP.4 Keypoint Descriptors

BRIEF, ORB, FREAK, AKAZE, and SIFT descriptors are implemented using OpenCV methods. Function definitions can be found in the `matching2D_Student.cpp` file.

### MP.5 Descriptor Matching

Both FLANN matching and k-nearest neighbor selection are implemented. Both methods are selectable using respective strings in the main function. Please refer to the implementation in `matching2D_Student.cpp`, specifically the function named `matchDescriptors`.

### MP.6 Descriptor Distance Ratio

K-Nearest-Neighbor matching with the descriptor distance ratio test is implemented. Please refer to the implementation in `matching2D_Student.cpp`, specifically the function named `matchDescriptors`.

## Performance Evaluation
### MP.7 Performance Evaluation 1

The table below shows the performance evaluation for different detector types:

| Detector Type       | Image-1  | Image-2  | Image-3  | Image-4  | Image-5  | Image-6  | Image-7  | Image-8  | Image-9  | Image-10 | Total Image Keypoints | Total ROI Keypoints | Avg Image Keypoints | Avg ROI Keypoints |
|---------------------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|-----------------------|---------------------|---------------------|-------------------|
| SHITOMASI           | 1370/125 | 1301/118 | 1361/123 | 1358/120 | 1333/120 | 1284/113 | 1322/114 | 1366/123 | 1389/111 | 1339/112 | 13423                 | 1179                | 1342.3              | 117.9             |
| HARRIS (with NMS)   | 115/17   | 98/14    | 113/18   | 121/21   | 160/26   | 383/43   | 85/18    | 210/31   | 171/26   | 281/34   | 1737                  | 248                 | 173.7               | 24.8              |
| FAST                | 1824/149 | 1832/152 | 1810/150 | 1817/155 | 1793/149 | 1796/149 | 1788/156 | 1695/150 | 1749/138 | 1770/143 | 17874                 | 1491                | 1787.4              | 149.1             |
| BRISK               | 2757/264 | 2777/282 | 2741/282 | 2735/277 | 2797/297 | 2695/279 | 2715/289 | 2628/272 | 2639/266 | 2672/254 | 27156                 | 2762                | 2715.6              | 276.2             |
| ORB                 | 500/92   | 500/102  | 500/106  | 500/113  | 500/109  | 500/125  | 500/130  | 500/129  | 500/127  | 500/128  | 5000                  | 1161                | 500                 | 116.1             |
| SIFT                | 1438/138 | 1371/132 | 1380/124 | 1335/137 | 1305/134 | 1370/140 | 1396/137 | 1382/148 | 1463/159 | 1422/137 | 13862                 | 1386                | 1386.2              | 138.6             |
| AKAZE               | 1351/166 | 1327/157 | 1311/161 | 1351/155 | 1360/163 | 1367/164 | 1363/173 | 1331/175 | 1357/177 | 1331/179 | 13449                 | 1670                | 1344.9              | 167               |

### MP.8 Performance Evaluation 2

The number of matched keypoints for all 10 images is counted using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8. The results are summarized in the following table:

| Detector\Descriptor  | BRISK | BRIEF | ORB  | FREAK | SIFT  | AKAZE |
|----------------------|-------|-------|------|-------|-------|-------|
| SHITOMASI            | 85    | 105   | 100  | 85    | 69    | Ref 3 |
| HARRIS (with NMS)    | 15    | 19    | 18   | 16    | 14    | Ref 3 |
| FAST                 | 100   | 122   | 119  | 97    | 81    | Ref 3 |
| BRISK                | 175   | 191   | 169  | 169   | 68    | Ref 3 |
| ORB                  | 83    | 62    | 85   | 46    | 47    | Ref 3 |
| SIFT                 | 65    | 79    | Ref 1, Ref 2 | 66 | 52    | Ref 3 |
| AKAZE                | 135   | 141   | 132  | 132   | 97    | 139   |

### MP.9 Performance Evaluation 3

The following table presents the total time (in ms) for various detector and descriptor combinations:

| Detector\Descriptor  | BRISK | BRIEF | ORB  | FREAK | SIFT  | AKAZE |
|----------------------|-------|-------|------|-------|-------|-------|
| SHITOMASI            | 14.08764 | 10.09 | 13.18 | 28.775 | 58.2574 | Ref 3 |
| HARRIS (with NMS)    | 14.6167  | 26.1491 | 10.576 | 31.312 | 24.08 | Ref 3 |
| FAST                 | 2.191 | 1.4381 | 1.205 | 22.397 | 13.169 | Ref 3 |
| BRISK                | 33.64 | 31.5395 | 33.933 | 62.5656 | 55.381 | Ref 3 |
| ORB                  | 6.215 | 57.27 | 8.108 | 26.421 | 94.143 | Ref 3 |
| SIFT                 | 143.346 | 187.668 | Ref 1, Ref 2 | 189.2714 | 222.939 | Ref 3 |
| AKAZE                | 169.238 | 279.406 | 198.139 | 210.234 | 208.662 | 394.56 |


References:
- [Ref 1](https://github.com/opencv/opencv/issues/10555)
- [Ref 2](https://knowledge.udacity.com/questions/989427)
- [Ref 3](https://docs.opencv.org/4.1.0/d8/d30/classcv_1_1AKAZE.html)

## Top 3 Recommendations

1. **FAST + BRIEF**:
   - **Speed and Efficiency**: Extremely low total runtime as observed in the table, making it excellent for real-time applications.
   - **High Keypoint Detection and Matching**: Large number of keypoints detected and high match count in SEL_KNN as observed in the table.

2. **FAST + ORB**:
   - **Rapid Detection with Robust Description**: Combines FAST's speed in detection with ORB's robustness in description.
   - **Good Performance**: Maintains high keypoint detection with effective descriptor matching.

3. **ORB + ORB**:
   - **Optimized Combination**: ORB is specifically designed to work efficiently with its own descriptor, as i know it from SLAM perspective, so this is more of a personal choice due to its robustness.
   - **Invariance Features**: Particularly effective in environments with variable scale and rotation.


   You can check results for some more experiments in the google sheet [here](https://docs.google.com/spreadsheets/d/1mIgZRGVAwiFfAWxsdUurBBcCHggUwDdHSfZpunv9tgE/edit?usp=sharing)



   ---


## Command-Line Convenience

My implementation allows you to specify parameters via the command line, making it easy to experiment with different configurations:

```sh
./2D_feature_tracking DETECTOR DESCRIPTOR MATCHER MATCHER_DESC SELECTOR
```

Available options:

**Detectors:** SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

**Descriptors:** BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

**Matcher Types:** MAT_BF (Brute Force), MAT_FLANN (FLANN-based)

**Matcher Type Descriptors:** DES_BINARY, DES_HOG

**Selector Types:** SEL_NN (Nearest Neighbor), SEL_KNN (K-Nearest Neighbors)

Example Usage

```
./2D_feature_tracking SHITOMASI BRISK MAT_BF DES_BINARY SEL_KNN;sleep 20; \
./2D_feature_tracking HARRIS BRISK MAT_BF DES_BINARY SEL_KNN;sleep 20; \
./2D_feature_tracking FAST BRISK MAT_BF DES_BINARY SEL_KNN;sleep 20; \
./2D_feature_tracking BRISK BRISK MAT_BF DES_BINARY SEL_KNN;sleep 20; \
./2D_feature_tracking ORB BRISK MAT_BF DES_BINARY SEL_KNN;sleep 20; \
./2D_feature_tracking SIFT BRISK MAT_BF DES_BINARY SEL_KNN;sleep 20; \
./2D_feature_tracking AKAZE BRISK MAT_BF DES_BINARY SEL_KNN
```


