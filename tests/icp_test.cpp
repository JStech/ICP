#include <iostream>
#include "icp.h"

using namespace pcl;

int main(int argc, char* argv[]) {
  std::vector<std::vector<float>> ref_pts = {
     {8.1472, 0.9754, 1.5761},
     {9.0579, 2.7850, 9.7059},
     {1.2699, 5.4688, 9.5717},
     {9.1338, 9.5751, 4.8538},
     {6.3236, 9.6489, 8.0028}
  };
  std::vector<std::vector<float>> src_pts = {
    { 8.2146,  3.5225,  2.5823},
    {10.2007,  4.4064, 10.6738},
    { 2.1986,  5.7739, 12.0774},
    { 8.5051, 11.6792,  6.9475},
    { 6.2768, 10.8903, 10.4448}
  };
  std::vector<int> matched = {0, 1, 2, 3, 4};

  PointCloud<PointXYZ> ref_cloud;
  PointCloud<PointXYZ> src_cloud;

  ref_cloud.width = 5;
  ref_cloud.height = 1;
  ref_cloud.is_dense = true;
  ref_cloud.points.resize(ref_cloud.width*ref_cloud.height);

  src_cloud.width = 5;
  src_cloud.height = 1;
  src_cloud.is_dense = true;
  src_cloud.points.resize(src_cloud.width*src_cloud.height);

  for (size_t i=0; i<ref_cloud.points.size(); i++) {
    ref_cloud.points[i].x = ref_pts[i][0];
    ref_cloud.points[i].y = ref_pts[i][1];
    ref_cloud.points[i].z = ref_pts[i][2];
    src_cloud.points[i].x = src_pts[i][0];
    src_cloud.points[i].y = src_pts[i][1];
    src_cloud.points[i].z = src_pts[i][2];
  }

  Eigen::Matrix<float, 4, 4> m = localize(ref_cloud, src_cloud, matched);

}
