#include "map_img_proccess/key_data_collect.h"
#include "map_img_proccess/image_match_fusion.h"

#include <thread>
#include <mutex>          // std::mutex

//namespace map_img_proccess
namespace key_data_collect
{   
  collect::collect(const std::string& scan_topic, const std::string& pose_topic, const std::string& keypoint_topic,const std::string& yaml_filename,const std::string& img_filename,const std::string& img_save_path)
  {
    std::cout<<"class collect created a object!"<<std::endl;
    //init object of ImageFusion 
    image_fusion_ = new image_match_fusion::ImageFusion(yaml_filename,img_filename,img_save_path);
    
    laser_pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laser_pose",100);
    laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n_,scan_topic,1);  
    pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n_,pose_topic,1);  
    keypoint_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(n_,keypoint_topic,1);
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(200), *laser_sub_, *pose_sub_, *keypoint_sub_);
    sync_->registerCallback(boost::bind(&collect::CallBack, this,  _1, _2,_3));
    
    // create image_fusion_thead
    image_process_id_ = new std::thread(&collect::ImageFusionThread,this);
    //create laserpose_sub_thread
    laserpose_sub_id_ = new std::thread(&collect::LaserPoseSubThread,this);
    //create capture input 
    capture_input_id_ = new std::thread(&collect::CaptureInputThread,this);
  }
  
  void collect::CallBack(const sensor_msgs::LaserScan::ConstPtr& laser_scan, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, const sensor_msgs::LaserScan::ConstPtr& keypoint_scan)
  {
    std::cout<<std::endl<<"callback:"<<laser_scan->header.stamp<<" "<<pose->header.stamp<<" "<<keypoint_scan->header.stamp<<std::endl;
    
    KeyDataType key_data_tmp;
    
    //collect laser data 
    LaserDataType laser_tmp;
    for(int i=0;i<laser_scan->ranges.size();i++)
    {
      if( std::isfinite(laser_scan->ranges[i]) &&  std::isnormal(laser_scan->ranges[i]) )
      {
	/////??????/////////////////////////////////////////////////////////////
	laser_tmp.x = -1.0 * laser_scan->ranges[i] * sin(i/180.0*3.14);
	laser_tmp.y =  laser_scan->ranges[i] * cos(i/180.0*3.14);
	key_data_tmp.laser_data.push_back(laser_tmp);
      }
    }
    //collect pose data 
    key_data_tmp.laser_pose.x = pose->pose.pose.position.x;
    key_data_tmp.laser_pose.y = pose->pose.pose.position.y;
    tf::Quaternion quat(pose->pose.pose.orientation.x,pose->pose.pose.orientation.y,pose->pose.pose.orientation.z,pose->pose.pose.orientation.w);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    key_data_tmp.laser_pose.theta = yaw;
    
    //collect keypoint data
    LaserDataType keypoint_tmp;
    for(int i=0;i<keypoint_scan->ranges.size();i++)
    {
      if( std::isfinite(keypoint_scan->ranges[i]) &&  std::isnormal(keypoint_scan->ranges[i]) )
      {
	/////??????/////////////////////////////////////////////////////////////
	keypoint_tmp.x =  keypoint_scan->ranges[i] * cos((i+63)/180.0*3.14);
	keypoint_tmp.y =  keypoint_scan->ranges[i] * sin((i+63)/180.0*3.14);
	key_data_tmp.keypoint_data.push_back(keypoint_tmp);
      }
    }
    
    //save to list
    std::lock_guard<std::mutex> lock(mtx_);
    data_list_.push_back(key_data_tmp);
    
    /*
    //debug
    KeyDataType out_tmp = data_list_.back();
    std::cout<<std::endl<<"laser:"<<std::endl;
    for(std::vector<LaserDataType>::iterator it = out_tmp.laser_data.begin();it < out_tmp.laser_data.end();it++)
    {
      std::cout<<"("<<it->x<<","<<it->y<<")";
    }
    std::cout<<std::endl<<"pose:"<<std::endl;
    std::cout<<"("<<out_tmp.laser_pose.x<<","<<out_tmp.laser_pose.y<<","<<out_tmp.laser_pose.theta<<")";
    std::cout<<std::endl<<"keypoint:"<<std::endl;
    for(std::vector<LaserDataType>::iterator it = out_tmp.keypoint_data.begin();it < out_tmp.keypoint_data.end();it++)
    {
      std::cout<<"("<<it->x<<","<<it->y<<")";
    }
    */
  }
  
  void collect::ImageFusionThread()
  {
    std::cout<<" ImageFusionThread run!"<<std::endl;
    while(true)
    {
      

      if(data_list_.empty())
      {
	//std::cout<<std::endl<<"data_list_ is empty!"<<std::endl;
	continue;
      }
      else
      {
	std::lock_guard<std::mutex> lock(mtx_);
	KeyDataType key_data_frame_temp = data_list_.back();
	//key_data_collect trans image_match_fusion
	image_match_fusion::KeyDataType key_data_frame_temp_trans;
	for(int i=0;i<key_data_frame_temp.laser_data.size();i++)
	{
	  image_match_fusion::LaserDataType laser_tmp;
	  laser_tmp.x = key_data_frame_temp.laser_data[i].x;
	  laser_tmp.y = key_data_frame_temp.laser_data[i].y; 
	  key_data_frame_temp_trans.laser_data.push_back(laser_tmp); 
	}
	key_data_frame_temp_trans.laser_pose.x = key_data_frame_temp.laser_pose.x;
	key_data_frame_temp_trans.laser_pose.y = key_data_frame_temp.laser_pose.y;
	key_data_frame_temp_trans.laser_pose.theta = key_data_frame_temp.laser_pose.theta;
	for(int i=0;i<key_data_frame_temp.keypoint_data.size();i++)
	{
	  image_match_fusion::LaserDataType keypoint_tmp;
	  keypoint_tmp.x = key_data_frame_temp.keypoint_data[i].x;
	  keypoint_tmp.y = key_data_frame_temp.keypoint_data[i].y; 
	  key_data_frame_temp_trans.keypoint_data.push_back(keypoint_tmp); 
	}
        data_list_.pop_back();
	
	image_fusion_->ShowImage(key_data_frame_temp_trans);
      }     
    }  
  }
  

  void collect::LaserPoseSubThread()
  {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::PoseWithCovarianceStamped laser_pose_msg;
    while(true)
    {
      try
      {
        listener.waitForTransform("/map", "/laser",ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/map", "/laser",ros::Time(0), transform);
	/*
	//debug
	std::cout<<std::endl<<"pose:"<<std::endl
	                                  <<"orig_x:"<<transform.getOrigin().x()<<std::endl
	                                  <<"orig_y:"<<transform.getOrigin().y()<<std::endl
	                                  <<"orig_z:"<<transform.getOrigin().z()<<std::endl
	                                  <<"rota_x:"<<transform.getRotation().getX()<<std::endl
	                                  <<"rota_y:"<<transform.getRotation().getY()<<std::endl
	                                  <<"rota_z:"<<transform.getRotation().getZ()<<std::endl
	                                  <<"rota_w:"<<transform.getRotation().getW()<<std::endl;
	*/
	laser_pose_msg.header.stamp = transform.stamp_;
	laser_pose_msg.pose.pose.position.x = transform.getOrigin().x();
	laser_pose_msg.pose.pose.position.y = transform.getOrigin().y();
	laser_pose_msg.pose.pose.position.z = transform.getOrigin().z();
	laser_pose_msg.pose.pose.orientation.x = transform.getRotation().getX();
	laser_pose_msg.pose.pose.orientation.y = transform.getRotation().getY();
	laser_pose_msg.pose.pose.orientation.z = transform.getRotation().getZ();
        laser_pose_msg.pose.pose.orientation.w = transform.getRotation().getW();
	laser_pose_pub_.publish(laser_pose_msg);
	ros::Duration(0.1).sleep();
	
      }
      catch(tf::TransformException &ex)
      {
	ROS_ERROR("%s",ex.what());
      }
    }
  }
  
  void collect::CaptureInputThread()
  {
    char ch;
    while(true)
    {
      std::cin>>ch;
      if(ch == 'q')
      {
	image_fusion_->finish_ = true;
	std::cout<<std::endl<<"map proccess finish!"<<std::endl;
	break;
      }
    }
    
  }
}//namespace key_data_collect
