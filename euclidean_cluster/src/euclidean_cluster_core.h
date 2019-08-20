#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>//条件欧式聚类分割

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>

#define LEAF_SIZE 0.1 //定义降采样的leaf size，聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
#define MIN_CLUSTER_SIZE 20 //聚类的最少点
#define MAX_CLUSTER_SIZE 5000 //聚类的最多点

class EuClusterCore
{

private:
  struct Detected_Obj   //定义一个结构体，用来存放检测到的障碍物信息
  {
    jsk_recognition_msgs::BoundingBox bounding_box_;

    pcl::PointXYZ min_point_;  //最小点
    pcl::PointXYZ max_point_;  //最大点 和最小点是对角线上的点
    pcl::PointXYZ centroid_;   //形心
  };

  ros::Subscriber sub_point_cloud_;

  ros::Publisher pub_bounding_boxs_;

  std::vector<double> seg_distance_, cluster_distance_;//存放分割阈值，存放分割成的5块点云的每一块的聚类阈值

  std_msgs::Header point_cloud_header_;

  void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);//定义体术滤波函数

  void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list);//定义根据距离将点云分成5块，方便进行不同阈值的欧式聚类

  void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                       double in_max_cluster_distance, std::vector<Detected_Obj> & obj_list);//对得到的5个点云根据半径的不同进行不同阈值的欧式聚类

  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);
/*
  void publish_cloud(const ros::Publisher &in_publisher,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                     const std_msgs::Header &in_header);//发布结果
*/
public:
  EuClusterCore(ros::NodeHandle &nh);//构造函数
  ~EuClusterCore();//析构函数
};