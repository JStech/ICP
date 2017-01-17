#ifndef ICP_H
#define ICP_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

float ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs, float D, float dt_thresh, float dth_thresh,
    int max_iter, std::vector<bool> *matched=NULL);

Eigen::Matrix4f localize(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
    pcl::PointCloud<pcl::PointXYZ>::Ptr source, std::vector<int> matched);

#endif
