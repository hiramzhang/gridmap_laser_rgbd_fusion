#ifndef KEY_DATA_COLLECT_H_
#define KEY_DATA_COLLECT_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<tf/LinearMath/Matrix3x3.h>
#include<tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include<math.h> //use isnan() isinf()
#include <limits> //use infinity()
#include<vector>
#include <thread>
#include <mutex>          // std::mutex

#include "map_img_proccess/image_match_fusion.h"


//namespace map_img_proccess
namespace key_data_collect
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,geometry_msgs::PoseWithCovarianceStamped,sensor_msgs::LaserScan> MySyncPolicy;
  
  struct LaserDataType
  {
    float x;
    float y;
  };
  struct LaserPoseType 
  {
    float x;
    float y;
    float theta;
  };
  struct KeyDataType
  {
    std::vector<LaserDataType> laser_data;
    LaserPoseType laser_pose;
    std::vector<LaserDataType> keypoint_data;    
  };
  
  class collect 
  {
  public:
    collect(const std::string& scan_topic, const std::string& pose_topic, const std::string& keypoint_topic,const std::string& yaml_filename,const std::string& img_filename,const std::string& img_save_path);
    ros::NodeHandle n_;
    std::vector<KeyDataType> data_list_;
    void CallBack(const sensor_msgs::LaserScan::ConstPtr& laser_scan, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, const sensor_msgs::LaserScan::ConstPtr& keypoint_scan);
    image_match_fusion::ImageFusion *image_fusion_;
    void ImageFusionThread();
    void LaserPoseSubThread();
    void CaptureInputThread();
    
  private:
    ros::Publisher laser_pose_pub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> *pose_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> *keypoint_sub_;
    message_filters::Synchronizer<MySyncPolicy> *sync_;
    std::thread *image_process_id_;
    std::thread *laserpose_sub_id_;
    std::thread *capture_input_id_;
    std::mutex mtx_; //lock data_list_ 
  };
}

#endif
