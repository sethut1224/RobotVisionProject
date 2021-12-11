#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv) 
{ 
  ros::init(argc, argv, "pseudo_lidar_tools");
  ros::NodeHandle nh;
  rosbag::Bag bag;
  int dist_thres = 30;
  bag.open("/home/autoware/shared_dir/bag/pseudo_lidar_rosbag" + std::to_string(dist_thres) + ".bag", rosbag::bagmode::Write);
  ros::Time stime = ros::Time::now();
  int cnt = 1;
  while (true)
  {
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << cnt << ".bin";
    std::string filename = ss.str();
    std::ifstream is_bin("/home/autoware/shared_dir/robot_vision/dataset/pseudo_lidar_result2/"+ filename, std::ifstream::binary);
    std::cout << "filename: " << filename << std::endl;
    if (is_bin) {
      // seekg를 이용한 파일 크기 추출
      
      is_bin.seekg(0, is_bin.end);
      int length = (int)is_bin.tellg();
      is_bin.seekg(0, is_bin.beg);

      // malloc으로 메모리 할당
      unsigned char * buffer = (unsigned char*)malloc(length);

      // read data as a block:
      is_bin.read((char*)buffer, length);
      is_bin.close();

      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      int current_length = 0;
      int points_size = length / 4 / 6;
      float *points_buff = (float *)buffer;
      double dist = 0.0;

      for (int i = 0; i < points_size; i++)
      {
        pcl::PointXYZRGB p;
        // p.x = points_buff[i * 6 + 0] * 256;
        // p.y = points_buff[i * 6 + 1] * 256;
        // p.z = points_buff[i * 6 + 2] * 256;
        p.x = points_buff[i * 6 + 2 ] * 256;
        p.y = -points_buff[i * 6 + 0] * 256;
        p.z = -points_buff[i * 6 + 1] * 256;
        dist = std::sqrt(std::pow(p.x,2) + std::pow(p.y,2) + std::pow(p.z, 2));
        // std::cout<<"dist : "<<dist<<std::endl;
        if(dist > dist_thres)
        {
          continue;
        }
        p.b = points_buff[i * 6 + 3];
        p.g = points_buff[i * 6 + 4];
        p.r = points_buff[i * 6 + 5];
        cloud.push_back(p);
        // std::cout << p.x << " " << p.y << " " << p.z << " " << points_buff[i * 6 + 3] << std::endl;
      }
      
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      ros::Time stamp = stime + ros::Duration(0.1 * cnt);
      cloud_msg.header.stamp = stamp;
      cloud_msg.header.frame_id = "/velodyne";
      bag.write("/points_raw", stamp, cloud_msg);
      
      // pcl::io::savePCDFile("/home/autoware/shared_dir/robot_vision/dataset/pcd_result/"+std::to_string(cnt)+".pcd", cloud);
      cnt++;
      free(buffer);
    }
    else
    {
      break;
    }

	}
  bag.close();

}
