#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

struct pair_hash {
    std::size_t operator()(const std::pair<int, int>& pair) const {
        return std::hash<int>()(pair.first) ^ (std::hash<int>()(pair.second) << 1);
    }
};

struct Node { // each cell defined by: position - actual cost (g) - extimated cost (f) - previous cell 
    int x, y;
    double g_cost, f_cost;
    int parent_x, parent_y;
    Node() : x(0), y(0), g_cost(0.0), f_cost(0.0), parent_x(-1), parent_y(-1) {}
    Node(int x_, int y_, double g_, double f_, int px, int py) : x(x_), y(y_), g_cost(g_), f_cost(f_), parent_x(px), parent_y(py) {}
    bool operator>(const Node& other) const { return f_cost > other.f_cost; }
};

ros::Publisher path_pub_euclidean;
ros::Publisher path_pub_chebyshev;
nav_msgs::OccupancyGrid current_costmap;
geometry_msgs::PoseStamped start_pose;
bool has_start = false;

double calculateHeuristic(int x1, int y1, int x2, int y2, std::string heuristic_type) {
    if (heuristic_type == "euclidean") {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    } else if (heuristic_type == "chebyshev") {
        return std::max(abs(x2 - x1), abs(y2 - y1));
    }
    return 0.0;
}

double getMovementCost(int x1, int y1, int x2, int y2) {
    if (x1 != x2 && y1 != y2) return 1.414;
    return 1.0;
}

bool isValidDiagonalMove(int x1, int y1, int x2, int y2, int width) {
    if (x1 == x2 || y1 == y2) return true;
    int idx1 = y2 * width + x1;
    int idx2 = y1 * width + x2;
    return !(current_costmap.data[idx1] >= 95 && current_costmap.data[idx2] >= 95);
}

std::vector<std::pair<int, int>> weightedAStarSearch(int start_x, int start_y, int goal_x, int goal_y, int width, int height, std::string heuristic_type) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::unordered_set<std::pair<int, int>, pair_hash> closed_list;
    std::unordered_map<int, std::unordered_map<int, Node>> all_nodes;

    double w = 2.0; 
    double h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y, heuristic_type);
    Node start_node(start_x, start_y, 0.0, h_cost, -1, -1);
    open_list.push(start_node);
    all_nodes[start_x][start_y] = start_node;

    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();

        if (current.x == goal_x && current.y == goal_y) {
            std::vector<std::pair<int, int>> path;
            Node curr = current;
            while (!(curr.x == start_x && curr.y == start_y)) {
                path.emplace_back(curr.x, curr.y);
                curr = all_nodes[curr.parent_x][curr.parent_y];
            }
            path.emplace_back(start_x, start_y);
            std::reverse(path.begin(), path.end());
            return path;
        }

        closed_list.insert({current.x, current.y});

        int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            if (closed_list.find({nx, ny}) != closed_list.end()) continue;

            int n_idx = ny * width + nx;
            int cell_cost = current_costmap.data[n_idx];
            
            if (cell_cost >= 95) continue; 
            if (!isValidDiagonalMove(current.x, current.y, nx, ny, width)) continue;

            double move_cost = getMovementCost(current.x, current.y, nx, ny);
            double mapped_costmap_cost = (cell_cost == -1) ? 0.0 : (double)cell_cost;
            
            double g_cost = current.g_cost + move_cost + mapped_costmap_cost;
            double new_h = calculateHeuristic(nx, ny, goal_x, goal_y, heuristic_type);
            double f_cost = g_cost + w * new_h;

            if (all_nodes.find(nx) == all_nodes.end() ||
                all_nodes[nx].find(ny) == all_nodes[nx].end() ||
                f_cost < all_nodes[nx][ny].f_cost) {

                Node neighbor(nx, ny, g_cost, f_cost, current.x, current.y);
                open_list.push(neighbor);
                all_nodes[nx][ny] = neighbor;
            }
        }
    }
    return {};
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_pose.header = msg->header;
    start_pose.pose = msg->pose.pose;
    has_start = true;
    ROS_INFO("NEW START RECEIVED");
}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    current_costmap = *msg;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!has_start || current_costmap.data.empty()) {
        ROS_ERROR("WAITING FOR COSTMAP");
        return;
    }

    int width = current_costmap.info.width;
    int height = current_costmap.info.height;
    double res = current_costmap.info.resolution;
    double origin_x = current_costmap.info.origin.position.x;
    double origin_y = current_costmap.info.origin.position.y;

    int start_x = std::floor((start_pose.pose.position.x - origin_x) / res);
    int start_y = std::floor((start_pose.pose.position.y - origin_y) / res);
    int goal_x = std::floor((msg->pose.position.x - origin_x) / res);
    int goal_y = std::floor((msg->pose.position.y - origin_y) / res);

    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height) {
        ROS_ERROR("START OR END BEYOND MAP"); 
        return;
    }

    ROS_INFO("COMPUTING A*");

    auto path_euclidean = weightedAStarSearch(start_x, start_y, goal_x, goal_y, width, height, "euclidean");
    auto path_chebyshev = weightedAStarSearch(start_x, start_y, goal_x, goal_y, width, height, "chebyshev");

    auto createPathMsg = [&](const std::vector<std::pair<int, int>>& points) {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& p : points) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.first * res + origin_x + res/2.0;
            pose.pose.position.y = p.second * res + origin_y + res/2.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        return path_msg;
    };

    if (!path_euclidean.empty()) {
        path_pub_euclidean.publish(createPathMsg(path_euclidean));
        ROS_INFO("EUCLIDEAN PATH FOUND");
    } 

    if (!path_chebyshev.empty()) {
        path_pub_chebyshev.publish(createPathMsg(path_chebyshev));
        ROS_INFO("CHEBYSHEV PATH FOUND");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    
    ros::Subscriber s1 = nh.subscribe("/initialpose", 1, startCallback);
    ros::Subscriber s2 = nh.subscribe("/move_base_simple/goal", 1, goalCallback);
    ros::Subscriber s3 = nh.subscribe("/processed_costmap", 1, costmapCallback);
    
    path_pub_euclidean = nh.advertise<nav_msgs::Path>("/planned_path_euclidean", 1, true);
    path_pub_chebyshev = nh.advertise<nav_msgs::Path>("/planned_path_chebyshev", 1, true);
    
    ROS_INFO("RUNNING path_planner");
    ros::spin();
    return 0;
}