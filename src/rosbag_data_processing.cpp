#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <vector>
#include <iostream>
#include <tf2_msgs::TFMessage>

using namespace std;

void 

int main (int argc, char **argv) {

  ros::init(argc, argv, "rosbag_data_processing_node");
  ros::NodeHandle n("~");
  ros::Publisher rviz_tf_pub = n.advertise<tf2_msgs::TFMessage>("out", 1000);
  ros::Subscriber tf_sub = n.subscribe<geometry_msgs/TransformStamped>("/tf", 50, tf_callback);
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("chatter"));
  topics.push_back(std::string("numbers"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const m, view)  {
    std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
  bag.close();
  return 0;
}
