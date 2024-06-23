#include <unordered_set>
#include <pcl/common/common.h>

template<typename PointT>
class QuizRansac {
private:
    int maxIterations;        // Maximum number of iterations for RANSAC algorithm
    float distanceThreshold;  // Distance threshold for considering a point as an inlier

public:
    // Constructor to initialize the RANSAC parameters
    QuizRansac(int maxIter, float distanceThreshold) : maxIterations(maxIter), distanceThreshold(distanceThreshold) {}

    // Destructor
    ~QuizRansac();

    // RANSAC algorithm to identify inliers in a 3D point cloud
    std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
};

template<typename PointT>
QuizRansac<PointT>::~QuizRansac() {} // Destructor definition

// Implementation of the RANSAC algorithm for 3D point clouds
template<typename PointT>
std::unordered_set<int> QuizRansac<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Record the start time for performance measurement
    auto startTime = std::chrono::steady_clock::now();

    // Set to store the indices of inliers
    std::unordered_set<int> inliersResult;
    srand(time(NULL)); // Seed for random number generation

    // Main RANSAC iteration loop
    while(maxIterations--)
    {
        // Set to keep track of current inliers
        std::unordered_set<int> inliers;
        while(inliers.size() < 3) // Ensure we sample 3 unique points
        {
            inliers.insert(rand() % (cloud->points.size()));
        }

        // Retrieve the coordinates of the three randomly picked points
        auto itr = inliers.begin();
        float x1 = cloud->points[*itr].x;
        float y1 = cloud->points[*itr].y;
        float z1 = cloud->points[*itr].z;
        itr++;
        float x2 = cloud->points[*itr].x;
        float y2 = cloud->points[*itr].y;
        float z2 = cloud->points[*itr].z;
        itr++;
        float x3 = cloud->points[*itr].x;
        float y3 = cloud->points[*itr].y;
        float z3 = cloud->points[*itr].z;

        // Calculate coefficients for the plane equation ax + by + cz + D = 0
        float a = ((y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1));
        float b = ((z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1));
        float c = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1));
        float D_coeff = -(a * x1 + b * y1 + c * z1);

        // Evaluate all other points against the plane equation
        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index) > 0) // Skip the points that define the plane
                continue;

            PointT point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            // Calculate the distance from the point to the plane
            float d = fabs(a * x4 + b * y4 + c * z4 + D_coeff) / sqrt(a*a + b*b + c*c);

            // If the distance is within the threshold, add point to inliers
            if (d <= distanceThreshold)
            {
                inliers.insert(index);
            }
        }

        // Update the result if the current set of inliers is the largest so far
        if(inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    // Record the end time and calculate the elapsed time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Manual Ransac for Plane took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return inliersResult;
}
