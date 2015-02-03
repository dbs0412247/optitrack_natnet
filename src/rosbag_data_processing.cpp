#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <vector>
#include <iostream>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>

using namespace std;

typedef struct {
  int head;
  int tail;
  trajectory_boundary(int h, int t) : head(h), tail(t) {  }
} trajectory_boundary;

void help() {
  cout << "Need at least one trajectory rosbag file!" << endl;
}

int main (int argc, char **argv) {

  if (argc < 2) {
    help();
    return -1;
  }
  if (!ros::ok()) 
    cout << "Waiting for master to start..." << endl;
  while(!ros::ok()) 
    ;

  ros::init(argc, argv, "rosbag_data_processing_node");
  ros::NodeHandle n("~");
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("trajectory_marker", 10);
  rosbag::Bag bag;
  vector< vector<geometry_msgs::TransformStamped> > vvTrajectory;
  int numCurrentTrajectory = 0;
  vector< trajectory_boundary > vBoundary;
  bool isFirstTrajectory = true;

  // read all trajectories
  for (int i = 1; i < argc; i++) {
    vector<geometry_msgs::TransformStamped> vTrajectory;
    bag.open(argv[1], rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (rosbag::View::iterator it = view.begin(); it != view.end(); it++)  {
      geometry_msgs::TransformStamped::ConstPtr ptr = it->instantiate<geometry_msgs::TransformStamped>();
      vTrajectory.push_back(*ptr);  
    }
    vvTrajectory.push_back(vTrajectory);
    vBoundary.push_back( trajectory_boundary(0, vTrajectory.size()-1) );
    bag.close();
  }
  // Debug 
  cout << "Size of vvTraj = " << vvTrajectory.size() << endl;

  uint32_t shape = visualization_msgs::Marker::POINTS;
  visualization_msgs::Marker points;
  points.header.frame_id = "/world";
  points.header.stamp = ros::Time::now();
  points.ns = "data_points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = shape;
  points.pose.position.x = 0;
  points.pose.position.y = 0;
  points.pose.position.z = 0;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.pose.orientation.w = 1.0;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.02;
  points.scale.y = 0.02;
  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;
  points.lifetime = ros::Duration();
    
  marker_pub.publish(points);
  while (ros::ok()) {
    if ( /* new command is received*/ || isFirstTrajectory) {
      // process command


      // re-publish data
      points.clear();
      for (int i = vBoundary[numCurrentTrajectory].head; i <= vBoundary[numCurrentTrajectory].tail; i++) {
        geometry_msgs::Point p;
        p.x = vvTrajectory[numCurrentTrajecory][i].transform.translation.x;
        p.y = vvTrajectory[numCurrentTrajecory][i].transform.translation.y;
        p.z = vvTrajectory[numCurrentTrajecory][i].transform.translation.z;
        points.points.push_back(p);
      }
      marker_pub.publish(points);
      if (isFirstTrajectory) 
        isFirstTrajectory = false;
    }
  }
  
  return 0;
}
