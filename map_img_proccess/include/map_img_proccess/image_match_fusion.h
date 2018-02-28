#ifndef IMAGE_MATCH_FUSION_H_
#define IMAGE_MATCH_FUSION_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "map_img_proccess/map_img_load.h"

const int N = 3;    //测试矩阵维数定义

namespace image_match_fusion 
{
  //input data type
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
  
  //data type of image transform to input type 
  //雷达扫描点的像素坐标系坐标
  struct LaserImageData
  {
    int x;
    int y;
  };
  //雷达位姿的像素坐标系坐标
  struct LaserPoseImageData 
  {
    int x;
    int y;
    float theta;
  };
  //图像像素坐标系数据结构
  struct ImageData
  {
    int x;
    int y;
  };
  //转移到图像坐标系后的子帧数据结构
  struct TransImageData
  {
    LaserPoseImageData laser_pose;
    std::vector<ImageData> laser_points;
    std::vector<ImageData> keypoints;
  };
  
  class ImageFusion 
  {
  public:
    ImageFusion(const std::string& yaml_filename,const std::string& img_filename,const std::string& img_save_path);
    TransImageData TransImage(KeyDataType key_data_frame);//世界坐标系到像素坐标系运算
    std::vector<double> VecDotMat(double vet[3],double matrix[3][3]);//3X3矩阵运算
    void ShowImage(KeyDataType key_data_frame);//把扫描的点投影到图像中
    ImageData WordToImage(const LaserPoseImageData laser_pose_image,
                          const LaserDataType word_points,double matrix[3][3]);


    bool finish_ ;//map save flag
    std::string image_save_path_;//image save path
    map_img_load::MapImgLoad  *map_img_load_;//类成员变量 图像加载类
    cv::Mat img_tmp_;
    int x_rows;
    int y_cols; 
    LaserPoseImageData word_coordinate_pixel_;//类成员变量 雷达位姿像素坐标点
    LaserPoseType  word_coordinate_;//类成员变量 雷达位姿世界坐标系点        
  };
}//namespace image_match_fusion 


#endif