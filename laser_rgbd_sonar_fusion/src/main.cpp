#include "laser_rgbd_sonar_fusion/scan_lrs.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_lr_node");
  scan_lrs::ScanFusion my_scan_fusion;
  ros::spin();
}