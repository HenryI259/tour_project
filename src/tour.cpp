#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/Image.h"
#include <cmath>
#include <random>
#include <queue>
#include <vector>
#include <utility>
#include <limits>
#include <algorithm>

using namespace std;

// Array of positions with x positions stored in even indices and y positions stored in odd
struct Node {
    double x;
    double y;

    Node(double x=0, double y=0) : x(x), y(y) {}
};

double euclidean_dis(Node n1, Node n2) {
    return hypot(n1.x - n2.x, n1.y - n2.y);
}

struct Path {
    vector<Node> nodes;
    double path_distance;

    void set_distance() {
        path_distance = 0;
        for (int i = 0; i < nodes.size()-1; i++) {
            path_distance += euclidean_dis(nodes[i], nodes[i+1]);
        }
    }
};

class WeightedGraph {
private:

public: 
    vector<Node> nodes;
    vector<vector<pair<int, double>>> edges;

    WeightedGraph(vector<Node> nodes): nodes(nodes) {
        edges.resize(nodes.size());
    }

    void add_edge(int u, int v) {
        double dis = euclidean_dis(nodes[u], nodes[v]);
        edges[u].push_back({v, dis});
        edges[v].push_back({u, dis});
    }

    Path dijkstra(int start_node, int dest_node) {
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
        vector<double> distances(nodes.size(), numeric_limits<double>::max());
        vector<int> parent(nodes.size(), -1);
        
        distances[start_node] = 0;
        pq.emplace(0.0, start_node);

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();

            int n = top.second;
            double dist = top.first;

            if (dist > distances[n])
                continue;
                
            for (auto& neighbor : edges[n]) {
                int v = neighbor.first;
                double w = neighbor.second;

                if (distances[n] + w < distances[v]) {
                    distances[v] = distances[n] + w;
                    parent[v] = n;
                    pq.emplace(distances[v], v);
                }
            }
        }

        vector<Node> path_nodes;
        int current_node = dest_node;
        while(current_node != -1) {
            path_nodes.push_back(nodes[current_node]);
            current_node = parent[current_node];
        }
        reverse(path_nodes.begin(), path_nodes.end());

        Path path_result;
        path_result.nodes = path_nodes;
        path_result.set_distance();
        return path_result;
    }

    Path tour(int start_node, vector<int> tour_nodes) {
        vector<int> remaining = tour_nodes;
        int current_node_index;
        // Find the start node set it as the beginning and mark it as removed nodes to choose
        for (int i = 0; i < remaining.size(); i++) {
            if (remaining[i] == start_node) {
                current_node_index = i;
            }
        }

        // Final tour path
        Path tour_path;
        tour_path.nodes.push_back(nodes[start_node]);
        for (int i = 0; i < remaining.size()-1; i++) {
            double min_dis = numeric_limits<double>::max();
            int next_node_index;
            // Find nearest neighbor
            Path min_inner_path;
            for (int j = 0; j < remaining.size(); j++) {
                if (remaining[j] != -1 && j != current_node_index) {
                    Path inner_path = dijkstra(remaining[current_node_index], remaining[j]);
                    if (inner_path.path_distance < min_dis) {
                        min_dis = inner_path.path_distance;
                        next_node_index = j;
                        min_inner_path = inner_path;
                    }
                }
            }

            // Add inner path to temp path
            for (int j = 1; j < min_inner_path.nodes.size(); j++) {
                tour_path.nodes.push_back(min_inner_path.nodes[j]);
            }
            
            // Mark node as removed and update index
            remaining[current_node_index] = -1;
            current_node_index  = next_node_index;
        }

        tour_path.set_distance();

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

    vector<Node> nodes;

    WeightedGraph* graph;

    Path tour_path;

    int current_node = 0;

    // Method for converting all angles to the range [0, 2pi]
    double correctAnglePos(double a) {
        while (a >= 6.28) {
            a -= 6.28;
        }
        while (a < 0) {
            a += 6.28;
        }
        return a;
    }

    // Method for converting all angles to the range [-pi, pi]
    double correctAngle(double a) {
        while (a >= 3.14) {
            a -= 6.28;
        }
        while (a < -3.14) {
            a += 6.28;
        }
        return a;
    }

    // Method for converting all angles to the range [-2pi, 0]
    double correctAngleNeg(double a) {
        while (a >= 0) {
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
        vector<Node> nodes(5);
        Node node1(0.0, 0.0);
        nodes[0] = node1;
        Node node2(1.0, 0.0);
        nodes[1] = node2;
        Node node3(1.0, 1.0);
        nodes[2] = node3;
        Node node4(0.0, 1.0);
        nodes[3] = node4;
        Node node5(0.5, 0.5);
        nodes[4] = node5;

        graph = new WeightedGraph(nodes);

        graph->add_edge(0, 1);
        graph->add_edge(1, 2);
        graph->add_edge(2, 3);
        graph->add_edge(3, 0);
        graph->add_edge(0, 4);
        graph->add_edge(1, 4);
        graph->add_edge(2, 4);
        graph->add_edge(3, 4); 
        
        vector<int> tour_nodes = {0, 2};
        int start_node = 0;

        tour_path = graph->tour(start_node, tour_nodes);

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

        // Flag for if the robot has finished the tour
        bool tour_finished = false;

        // init random generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distrib(-0.262, 0.262);

        // main loop
        while (ros::ok()) {
            printf("Current Node: %d / Target Node: (%.2f, %.2f) / Position: (%.2f, %.2f)\n", current_node, tour_path.nodes[current_node].x, tour_path.nodes[current_node].y, pos_x, pos_y, left_min, right_min);
            // Update node if reached
            if(euclidean_dis(Node(pos_x, pos_y), tour_path.nodes[current_node]) < 0.1) {
                if (current_node < tour_path.nodes.size() - 1) {
                    current_node++;
                }
                else {
                    tour_finished = true;
                }
            }
            
            // BASE BEHAVIOR
            linear_wire = 0;
            angular_wire = 0;
            
            // TRAVEL TO NODE BEHAVIOR
            double target_angle = atan2(tour_path.nodes[current_node].y - pos_y, tour_path.nodes[current_node].x - pos_x);
            double angle_diff = correctAngle(target_angle - angle);
            
            if (angle_diff > 0.1) {
                angular_wire = angular_speed;
            }
            else if (angle_diff < -0.1) {
                angular_wire = -angular_speed;
            }
            else {
                angular_wire = 0;
                linear_wire = linear_speed;
            }

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
            
            // Check if tour is finished
            if (tour_finished) {
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

    printf("Starting Robot Tour...\n");
    // Initialize robot
    robot.init();
    
    printf("Robot Tour Initialized.\n");
    // Run move method
    robot.move(pub, rate);

    return 0;
}

// Command to run robot
// roslaunch tour_project room_hallway_world_tour.launch
// roslaunch tour_project turtlebot_tour.launch
