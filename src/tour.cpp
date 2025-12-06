#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/Image.h"
#include <cmath>
#include <random>

using namespace std;

// Array of positions with x positions stored in even indices and y positions stored in odd
struct node {
    double x;
    double y;
};

struct path {
    int* nodes;
    int path_size;
    double path_distance;

    ~path() {
        delete[] nodes;
    }
};

class WeightedGraph {
private:
    double euclidean_dis(double x1, double y1, double x2, double y2) {
        return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
    }

    // TODO: Find distance along a path by adding the distances between nodes
    double path_distance(int path_size, int* path) {

    }

public: 
    node* nodes;
    int node_amount;
    double** edges;

    WeightedGraph(node* nodes, int node_amount, double** edges): nodes(nodes), node_amount(node_amount), edges(edges) {
        
    }

    ~WeightedGraph() {
        delete[] nodes;
        for (int i = 0; i < node_amount; i++) {
            delete[] edges[i];
        }
        delete[] edges;
    }

    // TODO: Implement A* that returns a path
    path Astar(int start_node, int dest_node) {
        
    }

    path tour(int start_node, int* tour_nodes, int tour_node_amount) {
        path** tour_paths = new path*[tour_node_amount];
        // Find all paths
        for(int i = 0; i < tour_node_amount; i++) {
            tour_paths[i] = new path[tour_node_amount];
            for (int j = 0; j < tour_node_amount; j++) {
                tour_paths[i][j] = Astar(tour_nodes[i], tour_nodes[j]);
            }
        }
        
        int current_node_index;
        // Find the start node set it as the beginning and mark it as removed nodes to choose
        for (int i = 0; i < tour_node_amount; i++) {
            if (tour_nodes[i] == start_node) {
                current_node_index = i;
                tour_nodes[i] = -1;
            }
        }

        // Final tour path
        path tour_path;
        tour_path.path_distance = 0;
        tour_path.path_size = 0;
        // Temp path nodes
        int tour_index = 0;
        int* path_nodes = new int[node_amount];
        for (int i = 0; i < tour_node_amount-1; i++) {
            double min_dis = 1000;
            int next_node_index;
            // Find nearest neighbor
            for (int j = 0; j < tour_node_amount; j++) {
                if (tour_nodes[j] != -1) {
                    if (tour_paths[current_node_index][j].path_distance < min_dis) {
                        min_dis = tour_paths[current_node_index][j].path_distance;
                        next_node_index = j;
                    }
                }
            }

            // Add inner path to temp path
            path inner_path = tour_paths[current_node_index][next_node_index];
            for (int j = 0; j < inner_path.path_size; j++) {
                path_nodes[tour_index] = inner_path.nodes[j];
                tour_index++;
            }

            // Update path size and distance
            tour_path.path_size += inner_path.path_size;
            tour_path.path_distance += inner_path.path_distance;
            
            // Mark node as removed and update index
            tour_nodes[next_node_index] = -1;
            current_node_index  = next_node_index;
        }

        // Copy over nodes so array is the correct size
        tour_path.nodes = new int[tour_path.path_size];
        for (int j = 0; j < tour_path.path_size; j++) {
            tour_path.nodes[j] = path_nodes[j];
        }

        // Free arrays
        for (int i = 0; i < tour_node_amount; i++) {
            delete[] tour_paths[i];
        }
        delete[] tour_paths;

        delete[] path_nodes;

        return tour_path;
    }
};

class ExplorerRobot {
private:

    // Position
    double pos_x;
    double pos_y;
    double angle;

    // Robot speed recieved from keyboard
    double keyboard_linear;
    double keyboard_angular;

    // Bumper info
    int bumper_side;
    int bumper_state;

    // Image Distances
    double right_min = 10;
    double left_min = 10;

    // Robot speeds
    double linear_speed = 0.3;
    double angular_speed = 1;

    int node_amount;
    node* nodes;
    double** edges;

    WeightedGraph* graph;

    int current_node = 0;

    // Method for converting all angles to the range [0, 2pi]
    double correctAnglePos(double a) {
        while (a > 6.28) {
            a -= 6.28;
        }
        while (a < 0) {
            a += 6.28;
        }
        return a;
    }

    // Method for converting all angles to the range [-2pi, 0]
    double correctAngleNeg(double a) {
        while (a > 0) {
            a -= 6.28;
        }
        while (a < -6.28) {
            a += 6.28;
        }
        return a;
    }

public:
    // Gets the postion
    void getPosition(const nav_msgs::Odometry::ConstPtr& msg) {
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
        geometry_msgs::Quaternion q = msg->pose.pose.orientation;
        // Calculate angle
        angle = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    // Gets the keyboard inputs
    void getInputs(const geometry_msgs::Twist::ConstPtr& msg) {
        keyboard_linear = msg->linear.x;
        keyboard_angular = msg->angular.z;
    }

    // Gets the bumper info
    void getBumper(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
        bumper_side = msg->bumper;
        bumper_state = msg->state;
    }
    // Gets Image data
    void getImage(const sensor_msgs::ImageConstPtr& msg) {
        int width = msg->width;
        int height = msg->height;
        int mid_row = height/2;
        int step = msg->step;

        right_min = 10;
        left_min = 10;

        // Encodings differ for each sensor
        if (msg->encoding == "32FC1") {
            const float* depth_data = reinterpret_cast<const float*>(&msg->data[0]);
            for (int r = mid_row - 2; r <= mid_row+2; r++) {
                for (int c = 0; c < width; c++) {
                    float d = depth_data[c + r*width]/2;
                    if (!std::isnan(d) && d > 0) {
                        if (c < width/2) {
                            if (d < left_min) {
                                left_min = d;
                            }
                        }
                        else{
                            if (d < right_min) {
                                right_min = d;
                            }
                        }
                    }
                }
            }
        }
        else if (msg->encoding == "16UC1") {
            const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(msg->data.data());
            for (int r = mid_row - 2; r <= mid_row+2; r++) {
                for (int c = 0; c < width; c++) {
                    float d = static_cast<float>(depth_data[c + r*width]) / 2000.0f;
                    if (!std::isnan(d) && d > 0) {
                        if (c < width/2) {
                            if (d < left_min) {
                                left_min = d;
                            }
                        }
                        else{
                            if (d < right_min) {
                                right_min = d;
                            }
                        }
                    }
                }
            }
        }
    }   

    void init() {
        node_amount;
        nodes = new node[node_amount];
        edges = new double*[node_amount]{
            new double[node_amount]{},
            new double[node_amount]{},
            new double[node_amount]{},
            new double[node_amount]{},
            new double[node_amount]{}
        };

        graph = new WeightedGraph(nodes, node_amount, edges);
        
        int tour_node_amount;
        int* tour_nodes = new int[tour_node_amount];
        int start_node;

        path tour_path = graph->tour(start_node, tour_nodes, tour_node_amount);

    }

    // move function
    void move(ros::Publisher pub, ros::Rate rate) {
        // Linear and angular value carried through the method
        double linear_wire;
        double angular_wire;

        // Stored positions and angles used for movement
        double start_x = pos_x;
        double start_y = pos_y;
        double start_angle = angle;

        // Flag for a fixed action turn
        bool uninterrupted_turn = false;

        float turning_angle = 0;

        // init random generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distrib(-0.262, 0.262);

        // main loop
        while (ros::ok()) {
            // BASE BEHAVIOR
            linear_wire = 0;
            angular_wire = 0;
            
            // TRAVEL TO NODE BEHAVIOR
            

            // AVOID ASYMETRIC OBJECTS BEHAVIOR
            if (left_min < 0.305) {
                angular_wire = -angular_speed;
                linear_wire = linear_speed/2;
            }
            else if (right_min < 0.305) {
                angular_wire = angular_speed;
                linear_wire = linear_speed/2;
            }

            
            // AVOID SYMETRIC OBJECTS BEHAVIOR
            if (left_min < 0.4 && right_min < 0.4 && !uninterrupted_turn) {
                turning_angle = -3.14;
                start_angle = angle;
                uninterrupted_turn = true;
            }
            
            // Don't move if turning
            if (uninterrupted_turn) {
                linear_wire = 0;
            }

            // Turn until the robot passes its target
            if (turning_angle > 0) {
                if (turning_angle > correctAnglePos(angle-start_angle)) {
                    angular_wire = angular_speed;
                }
                else {
                    turning_angle = 0;
                    uninterrupted_turn = false;
                }
            }
            else if (turning_angle < 0) {
                if (turning_angle < correctAngleNeg(angle-start_angle)) {
                    angular_wire = -angular_speed;
                }
                else {
                    turning_angle = 0;
                    uninterrupted_turn = false;
                }
            }

            // ACCEPT KEYBOARD INPUTS
            if (abs(keyboard_linear) > 0.01 || abs(keyboard_angular) > 0.01) {
                linear_wire = keyboard_linear;
                angular_wire = keyboard_angular;
                turning_angle = 0;
                uninterrupted_turn = false;
            }

            // HALT
            if (bumper_state) {
                linear_wire = 0;
                angular_wire = 0;
            }
            

            // Publish the message to the robot
            geometry_msgs::Twist vel_msg;

            vel_msg.linear.x = linear_wire;   
            vel_msg.angular.z = angular_wire;  
            
            pub.publish(vel_msg);

            // Retrieve data from the topics
            ros::spinOnce();
            rate.sleep();
        }
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_turtlebot2");

    ExplorerRobot robot;

    // Node Handler
    ros::NodeHandle nh;

    // Publisher to TurtleBot2 /mobile_base/commands/velocity
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    
    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, &ExplorerRobot::getPosition, &robot);
    ros::Subscriber teleop_sub = nh.subscribe("/my_teleop_node/cmd_vel", 10, &ExplorerRobot::getInputs, &robot);
    ros::Subscriber bumper_sub = nh.subscribe("/mobile_base/events/bumper", 10, &ExplorerRobot::getBumper, &robot);
    ros::Subscriber lidar_sub = nh.subscribe("/camera/depth/image_raw", 10, &ExplorerRobot::getImage, &robot);

    ros::Rate rate(60);

    // Run move method
    robot.move(pub, rate);

    return 0;
}

// Command to run robot
// roslaunch turtlebot_bringup minimal.launch
// roslaunch turtlebot_bringup 3dsensor.launch
// roslaunch tour_project room_hallway_world_tour.launch
// roslaunch tour_project turtlebot_tour.launch
