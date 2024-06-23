# Udacity Nanodegree Program Projects

## Projects

### 01. SFND Lidar Obstacle Detection
![Obstacle Detection](https://video.udacity-data.com/topher/2019/March/5c8599e3_obstacledetectionfps/obstacledetectionfps.gif)
<details>
  <summary>Description</summary>
    Implementation of custom RANSAC, KD-Tree, and Euclidean clustering algorithms as part of the processing pipeline for Lidar obstacle detection.
</details>

### 02. SFND 2D Feature Matching
![Keypoints](02_SFND_2D_Feature_Matching/images/keypoints.png)
<details>
  <summary>Description</summary>
    Implementation of various detectors, descriptors, and matching algorithms. It consists of four parts: data buffer, keypoint detection, descriptor extraction and matching, and performance evaluation.
</details>

### 03. SFND 3D Object Tracking
![3D Object Tracking](03_SFND_3D_Object_Tracking/assets/Fast-orb-perfect.png)
<details>
  <summary>Description</summary>
    Implementation of the following components:
    - Matching 3D objects
    - Computing Lidar-based TTC
    - Associating keypoint correspondences with bounding boxes
    - Computing Camera-based TTC
    - Performance evaluation
</details>

### 04. Radar Target Generation and Detection
![Radar Target Generation and Detection](04_Radar_target_generation_and_detection/assets/01_project_layout.png)
<details>
  <summary>Description</summary>
    Implementation of radar target generation and detection:
    - FMCW waveform design
    - Simulation loop
    - Range FFT (1st FFT)
    - 2D CFAR
</details>

### 05. SFND Unscented Kalman Filter
![Unscented Kalman Filter](05_SFND_Unscented_Kalman_Filter/media/ukf_highway.png)
<details>
  <summary>Description</summary>
  The simulation collects the position and velocity values output by the algorithm and compares them to the ground truth data. The px, py, vx, and vy RMSE values have been implemented to be less than or equal to [0.30, 0.16, 0.95, 0.70] after the simulator runs for more than 1 second. The simulator also displays if the RMSE values exceed the threshold.
</details>