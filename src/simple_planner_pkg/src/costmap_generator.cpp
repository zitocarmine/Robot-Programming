#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

ros::Publisher costmap_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    nav_msgs::OccupancyGrid costmap = *msg;
    int width = msg->info.width;
    int height = msg->info.height;
    int map_size = width * height;

    std::vector<int> distances(map_size);
    for(int i = 0; i < map_size; i++) {
        distances[i] = 999999;
    }
    std::queue<int> q;

    for (int i = 0; i < map_size; ++i) {
        if (msg->data[i] > 50) { 
            distances[i] = 0;
            q.push(i);
        }
    }

    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

    while (!q.empty()) {
        int curr = q.front();
        q.pop();

        int cx = curr % width;
        int cy = curr / width;

        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int neighbor_idx = ny * width + nx;
                int dist_to_add = (i < 4) ? 10 : 14; 

                if (distances[curr] + dist_to_add < distances[neighbor_idx]) {
                    distances[neighbor_idx] = distances[curr] + dist_to_add;
                    q.push(neighbor_idx);
                }
            }
        }
    }

    int safe_distance = 20; 
    
    for (int i = 0; i < map_size; ++i) {
        if (msg->data[i] > 50) {
            costmap.data[i] = 100;
        } else if (msg->data[i] == -1) {
            costmap.data[i] = -1; 
        } else {
            int dist = distances[i];
            if (dist >= safe_distance) {
                costmap.data[i] = 0;
            } else {
                int cost = 99 - (dist * 99 / safe_distance);
                costmap.data[i] = std::max(1, std::min(99, cost));
            }
        }
    }

    costmap_pub.publish(costmap);
    ROS_INFO("COSTMAP GENERATED");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_generator_node");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/processed_costmap", 1, true); 
    ros::spin();
    return 0;
}