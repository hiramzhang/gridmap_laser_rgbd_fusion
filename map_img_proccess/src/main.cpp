#include "map_img_proccess/key_data_collect.h"

#include <string>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "map_img_proccess_node");
  
  std::string laser_topic = "/scan";
  std::string pose_topic = "/laser_pose";
  std::string keypoint_topic = "/keypoint_scan";
  std::string map_yaml_dir = "/home/ubuntu/1_test.yaml";
  std::string map_pgm_dir = "/home/ubuntu/1_test.pgm";
  std::string map_save_dir = "/home/ubuntu/1_test_save.pgm";
  ros::param::get("~laser_topic", laser_topic);
  ros::param::get("~pose_topic", pose_topic);
  ros::param::get("~keypoint_topic", keypoint_topic);
  ros::param::get("~map_yaml_dir", map_yaml_dir);
  ros::param::get("~map_pgm_dir", map_pgm_dir);
  ros::param::get("~map_save_dir", map_save_dir);

  //key_data_collect::collect my_collect("/scan","/laser_pose","/keypoint_scan","/home/ubuntu/1_test.yaml","/home/ubuntu/1_test.pgm");
  key_data_collect::collect my_collect(laser_topic,pose_topic,keypoint_topic,map_yaml_dir,map_pgm_dir,map_save_dir);

  ros::spin();
  
  return 0;
}
