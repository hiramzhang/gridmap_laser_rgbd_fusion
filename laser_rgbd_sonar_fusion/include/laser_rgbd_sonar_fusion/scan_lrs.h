#ifndef SCAN_LRS_H_
#define SCAN_LRS_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h> //use isnan() isinf()
#include <limits> //use infinity()

namespace scan_lrs
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,sensor_msgs::LaserScan> MySyncPolicy;

class ScanFusion
{
public:
  ScanFusion();
  //void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_laser);
  void LaserXtionCallback(const sensor_msgs::LaserScan::ConstPtr& scan_laser,const sensor_msgs::LaserScan::ConstPtr& scan_xtion);
   //TODO:
  void Calibration(const sensor_msgs::LaserScan::ConstPtr& scan_in,sensor_msgs::LaserScan& scan_out);
  void KeypointExtra(const sensor_msgs::LaserScan::ConstPtr& scan_laser,sensor_msgs::LaserScan& scan_xtion);
    
private:
  ros::NodeHandle n_;
  //ros::Publisher scan_lr_pub_;
  ros::Publisher scan_keypoint_pub_;
  ros::Subscriber scan_laser_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *scan_l_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *scan_r_sub_;
  message_filters::Synchronizer<MySyncPolicy> *sync_;
  bool fusion_success_;//pub fusion resulte select
};
}//namespace scan_lrs

#endif
