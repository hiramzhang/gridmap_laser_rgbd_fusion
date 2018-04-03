#include "laser_rgbd_sonar_fusion/scan_lrs.h"

namespace scan_lrs
{
ScanFusion::ScanFusion()
{
  std::cout<<"created ScanFusion object!"<<std::endl;

  //scan_lr_pub_ = n_.advertise<sensor_msgs::LaserScan>("/scan",100);
  scan_keypoint_pub_ = n_.advertise<sensor_msgs::LaserScan>("/keypoint_scan",100);

  //scan_laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/laser_scan",1,&ScanFusion::LaserCallback,this);
  scan_l_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n_,"/scan",1);     
  scan_r_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n_,"/xtion_scan",1);
  sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *scan_l_sub_, *scan_r_sub_);
  sync_->registerCallback(boost::bind(&ScanFusion::LaserXtionCallback,this, _1, _2));

  fusion_success_ = false;
}

/*
void ScanFusion::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_laser)
{
  if(fusion_success_ == false)
    scan_lr_pub_.publish(*scan_laser);
}
*/

void ScanFusion::LaserXtionCallback(const sensor_msgs::LaserScan::ConstPtr& scan_laser,const sensor_msgs::LaserScan::ConstPtr& scan_xtion)
{
  std::cout<<std::endl<<"callback:"<<scan_laser->header.stamp<<" "<<scan_xtion->header.stamp<<std::endl;
  sensor_msgs::LaserScan scan_xtion_calib;
  Calibration(scan_xtion,scan_xtion_calib);
  KeypointExtra(scan_laser,scan_xtion_calib);
}

//TODO:
void ScanFusion::Calibration(const sensor_msgs::LaserScan::ConstPtr& scan_in,sensor_msgs::LaserScan& scan_out)
{
  //TODO:
  scan_out = *scan_in;
}

void ScanFusion::KeypointExtra(const sensor_msgs::LaserScan::ConstPtr& scan_laser,sensor_msgs::LaserScan& scan_xtion)
{
  //fill scan_laser buff
  float scan_laser_buff[360];
  for(int i=0;i<360;i++)
    scan_laser_buff[i] = scan_laser->ranges[i];
  //filter and cut point of scan_xtion
  float scan_xtion_buff[55];
  for(int i=0;i<55;i++)
  {
    float point_avg=0;
    int nan_num=0;
    //Local sampling and Mean filter 
    for(int j=-2;j<3;j++)
    {
      if( std::isfinite(scan_xtion.ranges[11*i+22+j]) &&  std::isnormal(scan_xtion.ranges[11*i+22+j]) )
        point_avg +=  scan_xtion.ranges[11*i+22+j];
      else
        nan_num += 1;
    }
    if(nan_num<3)
      point_avg = point_avg/(5-nan_num);
    else
      point_avg = std::numeric_limits<float>::infinity();
    
    scan_xtion_buff[i] = point_avg;
  }
  //scan_keypoint buff
  float scan_keypoint_buff[55];
  int keypoint_flag[55]; //to sum err_thres compute
  for(int i=0;i<55;i++)
  {
    scan_keypoint_buff[i] = std::numeric_limits<float>::infinity();
    keypoint_flag[i] = 0;
  }

  //fusion logic: input(scan_laser_buff,scan_xtion_buff)  ,output(scan_keypoint_buff)
  //*****************************************************************//
  //fusion param@
  //err_thres: 0.3
  //sum_err:  5    check>2
  //keypoint_num: check>10
  //xtion_view: -10~10
  const float err_thres = 0.5;
  const int sum_err_num = 17;
  const int sum_err_num_check = 13;
  const int keypoint_num_check = 15; 
  const int xtion_view = 15;
  for(int i=0;i<55;i++)
  {
    float laser_range_tmp;
    if(i<27)
      laser_range_tmp = scan_laser_buff[359-(26-i)];
    else
      laser_range_tmp = scan_laser_buff[i-27];

    if(std::isinf(laser_range_tmp) || std::isnan(laser_range_tmp))
    {
      scan_keypoint_buff[i] = scan_xtion_buff[i];
      keypoint_flag[i] = 1;
    }
    else if(std::isfinite(scan_xtion_buff[i]) && (laser_range_tmp-scan_xtion_buff[i]>err_thres))
    {
      scan_keypoint_buff[i] = scan_xtion_buff[i];
      keypoint_flag[i] = 1;
    }  
  }  
  //set keypoint view angle:-20~20
  for(int i=0;i<55;i++)
  {
    if(i<(-xtion_view+27) || i>(xtion_view+27))
      scan_keypoint_buff[i] = std::numeric_limits<float>::infinity();
    else //sum err check
    {
      int keypoint_flag_num = 0;
      for(int j=-1*(sum_err_num-1)/2;j<(sum_err_num+1)/2;j++)
        keypoint_flag_num += keypoint_flag[i+j];
      if(keypoint_flag_num<sum_err_num_check)
        scan_keypoint_buff[i] = std::numeric_limits<float>::infinity();
    }
  }
  //check keypoint num
  int num_tmp = 0;
  for(int i=0;i<55;i++)  
  {
    if( std::isfinite(scan_keypoint_buff[i]) )
      num_tmp++;
  }
  if(num_tmp>keypoint_num_check)
  {
    fusion_success_ = true;
  }
  //*****************************************************************//

  //pub scan_lr and scan_keypoint 
  if(fusion_success_ = true)
  {
    /*
    //pub scan_lr
    sensor_msgs::LaserScan scan_lr_msg;
    scan_lr_msg.header.stamp = scan_laser->header.stamp;
    scan_lr_msg.header.frame_id = "laser";
    scan_lr_msg.angle_min = scan_laser->angle_min;
    scan_lr_msg.angle_max = scan_laser->angle_max;
    scan_lr_msg.angle_increment = scan_laser->angle_increment;
    scan_lr_msg.time_increment = scan_laser->time_increment;
    scan_lr_msg.range_min = scan_laser->range_min;
    scan_lr_msg.range_max = scan_laser->range_max;
    scan_lr_msg.ranges.resize(360);
    for(int i=0;i<55;i++)
    {
      if(std::isfinite(scan_keypoint_buff[i]))
      {
        if(i<27)
          scan_laser_buff[359-(26-i)] = scan_keypoint_buff[i];
        else
          scan_laser_buff[i-27] = scan_keypoint_buff[i];
      }
    }
    for(int i=0;i<360;i++)
      scan_lr_msg.ranges[i] = scan_laser_buff[i];
    scan_lr_pub_.publish(scan_lr_msg);//execute pub
    */
    //pub scan_keypoint
    sensor_msgs::LaserScan scan_keypoint_msg;
    scan_keypoint_msg.header.stamp = scan_xtion.header.stamp;
    scan_keypoint_msg.header.frame_id = "keypoint_frame";
    scan_keypoint_msg.angle_min = -27/180.0*3.14;
    scan_keypoint_msg.angle_max = 27/180.0*3.14;
    scan_keypoint_msg.angle_increment = 1/180.0*3.14;
    scan_keypoint_msg.time_increment = scan_xtion.time_increment;
    scan_keypoint_msg.range_min = scan_xtion.range_min;
    scan_keypoint_msg.range_max = scan_xtion.range_max;
    scan_keypoint_msg.ranges.resize(55);
    for(int i=0;i<55;i++)
      scan_keypoint_msg.ranges[i] = scan_keypoint_buff[i];
    scan_keypoint_pub_.publish(scan_keypoint_msg);//execute pub
    //reset fusion_success_
    fusion_success_ = false;
  }
}

}//namespace scan_lrs

