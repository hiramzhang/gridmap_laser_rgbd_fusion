#ifndef MAP_IMG_LOAD_H_
#define MAP_IMG_LOAD_H_

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>

//namespace map_img_proccess 
namespace map_img_load
{
  struct ImgYaml
  {
    float resolution;
    float origin[3];
    float occupied_thresh;
    float free_thresh;
    int negate; 
  };
  
  class MapImgLoad 
  {
  public:
    MapImgLoad(const std::string& yaml_filename,const std::string& img_filename);
    void YamlRead();
    void ImgRead();
    
    std::string yaml_filename_;
    std::string img_filename_;
    ImgYaml img_yaml_;
    cv::Mat img_pgm_;
  };
}//namespace map_img_load

#endif

