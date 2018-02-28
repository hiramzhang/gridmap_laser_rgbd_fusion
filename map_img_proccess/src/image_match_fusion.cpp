#include "map_img_proccess/image_match_fusion.h"
#include "map_img_proccess/map_img_load.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


namespace image_match_fusion 
{
 

  ImageFusion::ImageFusion(const std::string& yaml_filename,const std::string& img_filename,const std::string& img_save_path)
  {
    std::cout<<"class ImageFusion created a object!"<<std::endl;
    
    map_img_load_ = new map_img_load::MapImgLoad(yaml_filename,img_filename);
    
    word_coordinate_pixel_.x = (int)( (-1.0 * map_img_load_->img_yaml_.origin[0])/map_img_load_->img_yaml_.resolution );
    word_coordinate_pixel_.y = (int)( (map_img_load_->img_pgm_.rows-((-1.0 * map_img_load_->img_yaml_.origin[1])/map_img_load_->img_yaml_.resolution)) );
    word_coordinate_pixel_.theta = 0.0;
    
    word_coordinate_.x = 0.0;
    word_coordinate_.y = 0.0;
    word_coordinate_.theta = 0.0;

    image_save_path_ = img_save_path;

  
    
    img_tmp_ = map_img_load_->img_pgm_.clone();
    finish_ = false;

    x_rows = img_tmp_.rows;
    y_cols = img_tmp_.cols;

    //std::cout<<"row:"<<map_img_load_->img_pgm_.rows<<std::endl;
    //std::cout<<"col:"<<map_img_load_->img_pgm_.cols<<std::endl;
    
    std::cout<<"row:"<<x_rows<<std::endl;
    std::cout<<"col:"<<y_cols<<std::endl;

    //cv::namedWindow("DisplayImage", CV_WINDOW_NORMAL);
    //cv::namedWindow("DisplayImage_keypoints", CV_WINDOW_NORMAL);
    }

    TransImageData ImageFusion::TransImage(KeyDataType key_data_frame)
    {
 
      double theta = 3.1415926-(key_data_frame.laser_pose.theta-1.5707963);
      std::cout<<"theta:"<<(key_data_frame.laser_pose.theta)*180/3.14<<std::endl;
      double matrix[3][3] = 
             {{cos(theta),-sin(theta),0.0},{sin(theta),cos(theta),0.0},{0.0,0.0,1.0}} ;//构建转移矩阵

      std::vector<LaserDataType> laser_data = key_data_frame.laser_data;
      LaserPoseType laser_pose = key_data_frame.laser_pose;
      std::vector<LaserDataType> keypoint_data = key_data_frame.keypoint_data;//子帧世界坐标系数据
	
      LaserPoseImageData laser_pose_image;
      std::vector<ImageData> laser_points_image;
      std::vector<ImageData> keypoints_image;//子帧像素坐标系数据
		
      //雷达位姿转换到像素坐标系
      int temp_img_pose[2];
      double vet_xy[2];
      LaserPoseImageData word_temp_pixel_pose;
      vet_xy[0] = (laser_pose.x -map_img_load_->img_yaml_.origin[0])/map_img_load_->img_yaml_.resolution;
      vet_xy[1] = (laser_pose.y -map_img_load_->img_yaml_.origin[1])/map_img_load_->img_yaml_.resolution;
      if(vet_xy[0]>=0 && vet_xy[0]<=x_rows 
        && vet_xy[1]>=0 && vet_xy[1]<=y_cols)
      {
        temp_img_pose[0] = int(vet_xy[0]);
        temp_img_pose[1] = int(map_img_load_->img_pgm_.rows - vet_xy[1]);
      }
      else
      {
        temp_img_pose[0] = 0;
        temp_img_pose[1] = 0;
      }
      
      /* //debug
      std::cout<<"pose_x:"<<key_data_frame.laser_pose.x<<std::endl;
      std::cout<<"pose_y:"<<key_data_frame.laser_pose.y<<std::endl;
      std::cout<<"map_img_load_->img_yaml_.origin[0]:"<<map_img_load_->img_yaml_.origin[0]<<std::endl;
      std::cout<<"map_img_load_->img_yaml_.origin[1]:"<<map_img_load_->img_yaml_.origin[1]<<std::endl;
      std::cout<<"laser_pose.x:"<<laser_pose.x<<std::endl;
      std::cout<<"laser_pose.y:"<<laser_pose.y<<std::endl;
      std::cout<<"vet_x:"<<vet_xy[0]<<std::endl;
      std::cout<<"vet_y:"<<vet_xy[1]<<std::endl;
      std::cout<<"temp_img_pose_tx:"<<temp_img_pose[0]<<std::endl;
      std::cout<<"temp_img_pose_ty:"<<temp_img_pose[1]<<std::endl;
      */
      word_temp_pixel_pose.x = temp_img_pose[0];
      word_temp_pixel_pose.y = temp_img_pose[1];
      laser_pose_image = word_temp_pixel_pose;
    
    //雷达点转换到像素坐标系
    int count_laser_data = laser_data.size();
    for (int i = 0; i < count_laser_data;i++)
    {	
	    
	    ImageData word_temp_pixel_laser;
	    word_temp_pixel_laser =  WordToImage(laser_pose_image,laser_data[i],matrix);
      //std::cout<<"word_temp_pixel_laser.x:"<<word_temp_pixel_laser.x<<std::endl;
      //std::cout<<"word_temp_pixel_laser.y:"<<word_temp_pixel_laser.y<<std::endl;
      if(word_temp_pixel_laser.x>=0 && word_temp_pixel_laser.x<=x_rows 
        && word_temp_pixel_laser.y>=0 && word_temp_pixel_laser.y<=y_cols)
      {
        laser_points_image.push_back(word_temp_pixel_laser); 
      }
      else
      {
        continue;
      }
      
    }
	
    //特征点转换到像素坐标系
    int count_keypoint_data = keypoint_data.size();
    for (int i = 0; i < count_keypoint_data;i++)
    {	
	    ImageData word_temp_pixel_keypoint;
      word_temp_pixel_keypoint = WordToImage(laser_pose_image,keypoint_data[i],matrix);
      //std::cout<<"word_temp_pixel_keypoint.x:"<<word_temp_pixel_keypoint.x<<std::endl;
      //std::cout<<"word_temp_pixel_keypoint.y:"<<word_temp_pixel_keypoint.y<<std::endl;
      
      if(word_temp_pixel_keypoint.x>=0 && word_temp_pixel_keypoint.x<=x_rows 
        && word_temp_pixel_keypoint.y>=0 && word_temp_pixel_keypoint.y<=y_cols)
      {
        keypoints_image.push_back(word_temp_pixel_keypoint);
      }
      else
      {
       continue;
      }

      
    }
	
    
      TransImageData return_data;
      return_data.laser_pose = laser_pose_image;
      return_data.laser_points = laser_points_image;
      return_data.keypoints = keypoints_image;
	
      return return_data;
    }
    
    //word to pixel
    ImageData ImageFusion:: WordToImage(
      const LaserPoseImageData laser_pose_image, const LaserDataType word_points,double matrix[3][3])
    {
      double temp_point[3];
      double vet_point_temp[2];
      std::vector<double> word_temp_keypoint;
      ImageData word_temp_pixel_point;
      temp_point[0] = -1.0*word_points.x;
      temp_point[1] = word_points.y;
      temp_point[2] = 1.0;
      word_temp_keypoint = VecDotMat(temp_point,matrix);
      vet_point_temp[0] = word_temp_keypoint[0]/map_img_load_->img_yaml_.resolution;
      vet_point_temp[1] = word_temp_keypoint[1]/map_img_load_->img_yaml_.resolution;

      word_temp_pixel_point.x = vet_point_temp[0] + laser_pose_image.x;
      word_temp_pixel_point.y = vet_point_temp[1] + laser_pose_image.y;
      return word_temp_pixel_point;

    }



    std::vector<double> ImageFusion::VecDotMat(double vet[3],double matrix[3][3])
    {
      std::vector<double> return_result;
      double result[3];
      result[0] = matrix[0][0] * vet[0] + matrix[0][1] * vet[1] + matrix[0][2] * vet[2];
      result[1] = matrix[1][0] * vet[0] + matrix[1][1] * vet[1] + matrix[1][2] * vet[2];
      result[2] = matrix[2][0] * vet[0] + matrix[2][1] * vet[1] + matrix[2][2] * vet[2];
      return_result.push_back(result[0]);
      return_result.push_back(result[1]);
      return_result.push_back(result[2]);
	
      return return_result;
    }
    
    void ImageFusion::ShowImage(KeyDataType key_data_frame)
    {
      std::cout<<"projection_submap_image run!"<<std::endl;
  	
      TransImageData transformation_coordinate_data;
      transformation_coordinate_data = TransImage(key_data_frame);
	    
      cv::Mat rgbImg(img_tmp_.size(),CV_8UC3);
      cv::cvtColor(img_tmp_, rgbImg, cv::COLOR_GRAY2BGR); 

      //画雷达中心位置点
      cv::Point point_Interest_laser_pose;
      point_Interest_laser_pose.x = transformation_coordinate_data.laser_pose.x;
      point_Interest_laser_pose.y = transformation_coordinate_data.laser_pose.y;
      cv::Point pStart(point_Interest_laser_pose.x, point_Interest_laser_pose.y);
      cv::circle(rgbImg, pStart, 6, cv::Scalar(255, 0, 0));
      
 
      //画origin位置点
      cv::Point point_Interest_laser_origin_pose;
      point_Interest_laser_origin_pose.x = word_coordinate_pixel_.x;
      point_Interest_laser_origin_pose.y = word_coordinate_pixel_.y;
      std::cout<<"origin_tx:"<<point_Interest_laser_origin_pose.x<<std::endl;
      std::cout<<"origin_ty:"<<point_Interest_laser_origin_pose.y<<std::endl;
      cv::circle(rgbImg, point_Interest_laser_origin_pose, 10, cv::Scalar(255, 255, 0));
      
      //画雷达扫描点
      std::cout<<"laser points"<<std::endl;
      int count_transformation_coordinate_data_laser_points = transformation_coordinate_data.laser_points.size();
      for (int i = 0; i < count_transformation_coordinate_data_laser_points;i++)
      {	
      cv::Point point_Interest_laser;
	    point_Interest_laser.x = transformation_coordinate_data.laser_points[i].x;
	    point_Interest_laser.y = transformation_coordinate_data.laser_points[i].y;
	    cv::circle(rgbImg, point_Interest_laser, 1, cv::Scalar(0, 255, 0));
      }
      //画特征点
      std::cout<<"keypoints"<<std::endl;
      int count_transformation_coordinate_data_keypoints = transformation_coordinate_data.keypoints.size();
      for (int i = 0; i < count_transformation_coordinate_data_keypoints;i++)
      {	
	    cv::Point point_Interest_laser_keypoint;
	    point_Interest_laser_keypoint.x = transformation_coordinate_data.keypoints[i].x;
	    point_Interest_laser_keypoint.y = transformation_coordinate_data.keypoints[i].y;
		//std::cout<<"111111111111111111"<<std::endl;
	    cv::circle(rgbImg, point_Interest_laser_keypoint, 2, cv::Scalar(0, 0, 255));	
            //cv::circle(img_tmp_, point_Interest_laser_keypoint, 0.95, cv::Scalar(0, 0, 0),1,8,0);
	    std::cout<<"point_Interest_laser_keypoint:"<<point_Interest_laser_keypoint<<std::endl;
      int pixel_gray = img_tmp_.at<uchar>(point_Interest_laser_keypoint.x,point_Interest_laser_keypoint.y);
	    std::cout<<"pixel_gray:"<<pixel_gray<<std::endl;
      pixel_gray -= 1;
	    if(pixel_gray < 0)
	      pixel_gray = 0;
	    cv::circle(img_tmp_, point_Interest_laser_keypoint, 0.95, cv::Scalar(pixel_gray),1,8,0);
      }
      //cv::namedWindow("DisplayImage", CV_WINDOW_NORMAL);
      //cv::namedWindow("DisplayImage_keypoints", CV_WINDOW_NORMAL);
      cv::imshow("DisplayImage", rgbImg);
      cv::imshow("DisplayImage_keypoints", img_tmp_);
      cv::waitKey(10);
      //cv::destroyWindow("DisplayImage");
      //cv::destroyWindow("DisplayImage_keypoints");
      if (finish_)
      {
        cv::imwrite(image_save_path_,img_tmp_);
      }
   }
}//namespace image_match_fusion
