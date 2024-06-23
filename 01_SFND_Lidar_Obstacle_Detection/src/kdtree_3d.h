// Structure to represent node of kd tree
struct Node
{
    pcl::PointXYZI point; // 3D point represented by this node
    int id;               // Unique identifier for the point
    Node* left;           // Pointer to left child in KD-Tree
    Node* right;          // Pointer to right child in KD-Tree

    // Constructor to initialize a KD-Tree node with a point and id
    Node(pcl::PointXYZI arr, int setId)
    :   point(arr), id(setId), left(NULL), right(NULL)
    {}
};

struct KdTree
{
    Node* root; // Root of the KD-Tree

    // Constructor to initialize an empty KD-Tree
    KdTree()
    : root(NULL)
    {}

    // Helper function to recursively insert a new point into the KD-Tree
    void insertHelper(Node **node, int depth, pcl::PointXYZI point, int id) {
        // Create a new node if current node is null
        if (*node == NULL) {
            *node = new Node(point, id);
        }
        else {
            // Calculate current dimension (0 for x, 1 for y, 2 for z) based on depth
            int ComparisonDimension = depth % 3;

            // Recursively insert the point in left or right subtree based on comparison with current dimension
            if (ComparisonDimension == 0) { // Compare x-coordinates
                if (point.x < (*node)->point.x)
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            }
            else if (ComparisonDimension == 1) { // Compare y-coordinates
                if (point.y < (*node)->point.y)
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            }
            else { // Compare z-coordinates
                if (point.z < (*node)->point.z)
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            }
        }
    }

    // Public method to insert a new point into the KD-Tree
    void insert(pcl::PointXYZI point, int id)
    {
        // Start insertion from the root at depth 0
        insertHelper(&root, 0, point, id);
    }

    // Helper function to search for points within a certain distance from a target point
    void searchHelper(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int> &ids) {
        if (node != NULL) {
            // Calculate deltas for x, y, z coordinates
            float delta_x = node->point.x - target.x;
            float delta_y = node->point.y - target.y;
            float delta_z = node->point.z - target.z;

            // Check if the node point is within the box defined by distanceTol
            if ((-distanceTol <= delta_x && delta_x <= distanceTol) &&
                (-distanceTol <= delta_y && delta_y <= distanceTol) &&
                (-distanceTol <= delta_z && delta_z <= distanceTol)) {
                // Calculate distance from target to the point
                float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
                // Add point to result if it's within distanceTol
                if (distance <= distanceTol) {
                    ids.push_back(node->id);
                }
            }

            // Recursively search left and right children considering the dimension based on depth
            if (depth % 3 == 0) { // Check x-coordinates
                if (-distanceTol < delta_x)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if (distanceTol > delta_x)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            else if (depth % 3 == 1) { // Check y-coordinates
                if (-distanceTol < delta_y)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if (distanceTol > delta_y)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            else { // Check z-coordinates
                if (-distanceTol < delta_z)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if (distanceTol > delta_z)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
        }
    }

    // Public method to search for points within a certain distance from a target point
    std::vector<int> search(pcl::PointXYZI target, float distanceTol) {
        std::vector<int> ids; // Vector to store ids of points within distanceTol from the target
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};
