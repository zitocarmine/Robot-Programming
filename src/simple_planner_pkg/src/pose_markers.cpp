#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher marker_pub;
visualization_msgs::Marker initial_pose_marker;
visualization_msgs::Marker goal_pose_marker;

visualization_msgs::Marker createMarker(int id, const std::string& ns, float r, float g, float b) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.4; marker.scale.y = 0.4; marker.scale.z = 0.4; 
    marker.color.a = 1.0; marker.color.r = r; marker.color.g = g; marker.color.b = b;
    return marker;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    initial_pose_marker.header.stamp = ros::Time::now();
    initial_pose_marker.pose.position = msg->pose.pose.position;
    initial_pose_marker.pose.orientation.w = 1.0;
    marker_pub.publish(initial_pose_marker);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_pose_marker.header.stamp = ros::Time::now();
    goal_pose_marker.pose.position = msg->pose.position;
    goal_pose_marker.pose.orientation.w = 1.0;
    marker_pub.publish(goal_pose_marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_markers");
    ros::NodeHandle nh;
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("/pose_markers", 1);
    
    initial_pose_marker = createMarker(0, "initial_pose", 0.0f, 1.0f, 0.0f); 
    goal_pose_marker = createMarker(1, "goal_pose", 1.0f, 0.0f, 0.0f);       
    
    ros::Subscriber sub1 = nh.subscribe("/initialpose", 1, initialPoseCallback);
    ros::Subscriber sub2 = nh.subscribe("/move_base_simple/goal", 1, goalCallback);
    
    ROS_INFO("RUNNING pose_markers. WAINTING FOR POSITIONS");
    ros::spin();
    return 0;
}