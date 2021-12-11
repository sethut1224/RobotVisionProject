#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv) 
{   
    std::string cloud_dir = "/home/autoware/shared_dir/robot_vision/dataset/pcd_result/";
    std::ifstream is("/home/autoware/shared_dir/poses/10.txt");
    std::vector<Eigen::Matrix4f> poses;
    
    while (true)
    {
        Eigen::Matrix4f p;
        // is >> p(0, 0); is >> p(0, 1); is >> p(0, 2); is >> p(0, 3);
        // is >> p(1, 0); is >> p(1, 1); is >> p(1, 2); is >> p(1, 3); 
        // is >> p(2, 0); is >> p(2, 1); is >> p(2, 2); is >> p(2, 3);
        // is >> p(3, 0); is >> p(3, 1); is >> p(3, 2); is >> p(3, 3);
        is >> p(0, 0); is >> p(0, 1); is >> p(0, 2); is >> p(0, 3);
        is >> p(1, 0); is >> p(1, 1); is >> p(1, 2); is >> p(1, 3); 
        is >> p(2, 0); is >> p(2, 1); is >> p(2, 2); is >> p(2, 3);
        p(3, 0) = 0.0;
        p(3, 1) = 0.0;
        p(3, 2) = 0.0;
        p(3, 3) = 1.0;
        poses.push_back(p);
        if (is.eof())
            break;
    }
    std::cout << "poses.size(): " << poses.size() << std::endl;
    poses.pop_back();
    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    int seq = 0;
    while (true)
    {
        std::cout << "current seq: " << seq << std::endl;
        pcl::PointCloud<pcl::PointXYZ> cloud, tfd_cloud;
        if (pcl::io::loadPCDFile(cloud_dir + std::to_string(seq) + ".pcd", cloud) < 0)
        {
            break;
        }
        
        pcl::transformPointCloud(cloud, tfd_cloud, poses[seq]);
        map_cloud += tfd_cloud;

        pcl::io::savePCDFile("/home/autoware/shared_dir/robot_vision/dataset/map_pcd_result/" + std::to_string(seq) + ".pcd", tfd_cloud, true);
        seq++;
    }

    pcl::io::savePCDFile("/home/autoware/shared_dir/robot_vision/dataset/map.pcd", map_cloud, true);
    return 0;
}
