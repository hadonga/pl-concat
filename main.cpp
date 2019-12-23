#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>  
#include <unistd.h>  
#include <dirent.h>
#include <vector> 

using namespace std;
using namespace pcl;

// class cancellation {
//   	void insertPoints(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud) {
// 		bin_index_.resize(cloud->size());
// 		double segment_step = 2 * M_PI / n_segment;
// 		double bin_step = (sqrt(r_max_square) - sqrt(r_min_square)) / n_bin;
// 		double r_min = sqrt(r_min_square);

// 		for (int i = 0; i < cloud->size(); ++i) {
// 			double range_square = cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y;
// 			double range = sqrt(range_square);

// 			if (range_square <r_max_square && range_square > r_min_square) {
// 				//double angle = std::atan2(cloud->points[i].y, cloud->points[i].x);
// 				unsigned int bin_index = (range - r_min) / bin_step;
// 				unsigned int segment_index = floor((std::atan2(cloud->points[i].y, cloud->points[i].x) + M_PI) / (2 * M_PI / n_segment));
// 				//cout << "bin_index:" << bin_index << "----segment_index:" << segment_index << endl;
// 				if (bin_index > n_bin - 1) { bin_index = bin_index % n_bin; }
// 				if (segment_index > n_segment - 1) { segment_index = segment_index % n_segment; }
// 				segments_[segment_index].bins_[bin_index].addPoint(cloud->points[i]);
// 				bin_index_[i] = std::make_pair(segment_index, bin_index);
// 			}
// 			else {
// 				bin_index_[i] = std::make_pair<int, int>(-1, -1);
// 			}

// 		}

// 	}
// return cloud};

int main (int argc, char** argv)
{
  vector <string> files;
	DIR *dir;
	struct dirent *ptr;
	char base[1000];
  const int count = 25000;
  string pl_path="./pcd_2hrs/";

  if ((dir=opendir("./pcd_2hrs"))==NULL){
		perror("No such dir ... ");
		exit(1);
	}
	while ((ptr=readdir(dir))!=NULL){
		if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir  
                continue;  
        else if(ptr->d_type == 8)    ///file   
            files.push_back(ptr->d_name);  
        else if(ptr->d_type == 10)    ///link file  
            continue;  
        else if(ptr->d_type == 4)    ///dir  
        {  
            files.push_back(ptr->d_name);  
        }  
	}
	closedir(dir);
  for (int i=0;i<count;i++){
	cout<<"loading "<<files[i]<<"-->"<< i <<"\n";
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRes (new pcl::PointCloud<pcl::PointXYZI>); // 创建Result点云（指针）
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOrg (new pcl::PointCloud<pcl::PointXYZI>); // 创建Org点云（指针）
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAdd (new pcl::PointCloud<pcl::PointXYZI>); // Add Point Cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOpt (new pcl::PointCloud<pcl::PointXYZI>); 


 if (pcl::io::loadPCDFile<pcl::PointXYZI> (pl_path+files[0], *cloudOrg) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
    PCL_ERROR ("Couldn't read PCD files \n"); //文件不存在时，返回错误，终止程序。
    return (-1);
  }
  
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pl_path+files[1], *cloudAdd) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
    PCL_ERROR ("Couldn't read PCD files \n"); //文件不存在时，返回错误，终止程序。
    return (-1);
  }
  *cloudRes=*cloudOrg+*cloudAdd;
  
  for(int pl=2;pl<count;pl++){
  if (pl%50==0){
  cout << "Concatenating the " << pl <<"th Point Cloud ... \n";}
  cloudAdd->clear();
  cloudOpt->clear();
  
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pl_path+files[pl], *cloudAdd) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
    PCL_ERROR ("Couldn't read PCD files \n"); //文件不存在时，返回错误，终止程序。
    return (-1);
  }
  else{
    for(int i=0;i< cloudAdd->size();i++){     
      if ((cloudAdd->points[i].x >-3 && cloudAdd->points[i].x<6 && cloudAdd->points[i].y>-9 && cloudAdd->points[i].y<0.6)==false){
        cloudOpt->push_back(cloudAdd->points[i]);
      }
    }
  }
  *cloudRes=*cloudRes+*cloudOpt;
//  *cloudRes = cancellation(cloudRes);
  }

  pcl::io::savePCDFile("CloudRes.pcd",*cloudRes);
  std::cout << "Loaded "
            << cloudRes->width * cloudRes->height
            << " data points from test_file.pcd with the following fields: "
            << std::endl;
  //for (size_t i = 0; i < cloud->points.size (); ++i) //显示所有的点
	for (size_t i = 0; i < 10; ++i) // 为了方便观察，只显示前5个点
    std::cout << "    " << cloudRes->points[i].x
              << " "    << cloudRes->points[i].y
              << " "    << cloudRes->points[i].z
              << " "    << cloudRes->points[i].intensity 
              << std::endl;

  return (0);
}