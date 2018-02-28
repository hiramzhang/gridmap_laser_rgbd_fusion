#include "map_img_proccess/map_img_load.h"

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//namespace map_img_proccess
namespace map_img_load
{
  MapImgLoad::MapImgLoad(const std::string& yaml_filename,const std::string& img_filename):
                                                                                                 yaml_filename_(yaml_filename),
                                                                                                 img_filename_(img_filename)
  {
    std::cout<<"class MapImgLoad created a object!"<<std::endl;
    
    this->YamlRead();
    this->ImgRead();
  }
  
  void MapImgLoad::YamlRead()
  {
    cv::FileStorage fs(yaml_filename_, cv::FileStorage::READ);
    fs["resolution"] >> img_yaml_.resolution;
    cv::FileNode orig = fs["origin"];
    int indx = 0;
    for(cv::FileNodeIterator it = orig.begin(); it != orig.end(); ++it,indx++)
      *it >> img_yaml_.origin[indx];
    fs["occupied_thresh"] >> img_yaml_.occupied_thresh;
    fs["free_thresh"] >> img_yaml_.free_thresh;
    fs["negate"] >> img_yaml_.negate;
    fs.release();
    
    std::cout<<"resolution: "<<img_yaml_.resolution<<std::endl
                   <<"origin: "<<"["<<img_yaml_.origin[0]<<" "<<img_yaml_.origin[1]<<" "<<img_yaml_.origin[2]<<"]"<<std::endl
                   <<"occupied_thresh: "<<img_yaml_.occupied_thresh<<std::endl
                   <<"free_thresh: "<<img_yaml_.free_thresh<<std::endl
                   <<"negate: "<<img_yaml_.negate<<std::endl;                 
  }
  
  void MapImgLoad::ImgRead()
  {
    img_pgm_ = cv::imread(img_filename_, CV_LOAD_IMAGE_UNCHANGED);
    if(img_pgm_.empty())
    {
        std::cout << "load img failed!"<< std::endl;
    }
    else
    {
      std::cout << "load img success!"<< std::endl;
      /*
      cv::namedWindow("MyWindow", CV_WINDOW_AUTOSIZE);
      cv::imshow("MyWindow", img_pgm_);
      
      
      //debug
      cv::Mat img_tmp = img_pgm_.clone();
      int orig_pixel_x =(int)( img_tmp.rows - (0 - img_yaml_.origin[1]) / img_yaml_.resolution );
      int orig_pixel_y =(int)( (0 - img_yaml_.origin[0]) / img_yaml_.resolution );
      for(int x=-5;x<5;x++)
	for(int y=-5;y<5;y++)
	  img_tmp.at<uchar>(orig_pixel_x+x,orig_pixel_y+y)=0;
      cv::namedWindow("MyWindow2", CV_WINDOW_AUTOSIZE);
      cv::imshow("MyWindow2", img_tmp);
      std::cout<<"rows： "<<img_tmp.rows<<std::endl<<"cols: "<<img_tmp.cols<<std::endl;
      
      cv::waitKey(3000);
      cv::destroyWindow("MyWindow");
      cv::destroyWindow("MyWindow2");
      */
      
    }
  }
  
}//namespace map_img_load
