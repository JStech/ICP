#ifndef ICP_H
#define ICP_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

float ICP_hmrf(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs, float beta, float dt_thresh,
    float dth_thresh, int max_iter, std::vector<bool> *matched=NULL);

float ICP_zhang(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs, float D, float dt_thresh, float dth_thresh,
    int max_iter, std::vector<bool> *matched=NULL);

Eigen::Matrix4f localize(pcl::PointCloud<pcl::PointXYZ>::Ptr reference,
    pcl::PointCloud<pcl::PointXYZ>::Ptr source, std::vector<int> matched,
    bool do_scale);

void downsample_cloud(float d,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud);

#endif
