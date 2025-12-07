#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/Image.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
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
    double pos_x = 0, pos_y = 0;

    WeightedGraph *graph;
    Path tour_path;
    int current_node = 0;

public:

    bool ready = false;

    void getPosition(const nav_msgs::Odometry::ConstPtr &msg) {
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
    }

    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        const boost::array<double, 36> &cov = msg->pose.covariance;  // use as array
        double max_cov = std::max({cov[0], cov[7], cov[35]});  // x, y, yaw
        if (max_cov < 0.5) {  // threshold, adjust as needed
            ready = true;
        }
    }

    void init() {
        vector<Node> nodes(5);
        nodes[0] = Node(0,0);
        nodes[1] = Node(5,0);
        // nodes[2] = Node(2,2);
        // nodes[3] = Node(0,2);
        // nodes[4] = Node(1,1);

        graph = new WeightedGraph(nodes);

        graph->add_edge(0,1);
        // graph->add_edge(1,2);
        // graph->add_edge(2,3);
        // graph->add_edge(3,0);
        // graph->add_edge(0,4);
        // graph->add_edge(1,4);
        // graph->add_edge(2,4);
        // graph->add_edge(3,4);

        vector<int> tour_nodes = {0, 1};
        tour_path = graph->tour(0, tour_nodes);
    }

    bool goToNode(Node target, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> &ac) {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = target.x;
        goal.target_pose.pose.position.y = target.y;
        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal (%.2f, %.2f)...", target.x, target.y);
        ac.sendGoal(goal);

        bool finished = ac.waitForResult(ros::Duration(300.0));

        if (!finished) {
            ROS_WARN("Navigation timeout.");
            ac.cancelGoal();
            return false;
        }

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Reached (%.2f, %.2f).", target.x, target.y);
            return true;
        }

        ROS_WARN("Failed to reach (%.2f, %.2f).", target.x, target.y);
        return false;
    }

    // -------- MOVE = send all nodes sequentially --------

    void run() {
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        ROS_INFO("Waiting for move_base action server...");
        ac.waitForServer();

        for (int i = 0; i < tour_path.nodes.size(); i++) {
            Node n = tour_path.nodes[i];
            ROS_INFO("NAVIGATING TO NODE #%d...", i);

            bool ok = goToNode(n, ac);
            if (!ok) {
                ROS_WARN("Could not reach node %d, skipping.", i);
                continue;
            }
        }

        ROS_INFO("TOUR COMPLETE.");
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_tour_navstack");
    ros::NodeHandle nh;

    ExplorerRobot robot;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, &ExplorerRobot::getPosition, &robot);
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 10, &ExplorerRobot::amclPoseCallback, &robot);

    robot.init();

    ROS_INFO("Waiting for AMCL pose estimate...");
    ros::Rate rate(10); // 10 Hz
    while (ros::ok() && !robot.ready) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("AMCL pose ready, starting tour.");

    ROS_INFO("Starting tour with Navigation Stack.");
    robot.run();

    return 0;
}

// Command to run robot
// roslaunch tour_project room_hallway_world_tour.launch
// roslaunch tour_project turtlebot_tour.launch
