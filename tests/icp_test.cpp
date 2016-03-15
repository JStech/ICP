#include <iostream>
#include "icp.h"

using namespace pcl;

int main(int argc, char* argv[]) {
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
    ref_cloud.points[i].x = (float) i;
    ref_cloud.points[i].y = ((float) i)*((float) i);
    ref_cloud.points[i].z = 1. / (3. + (float) i);
    src_cloud.
  }

}
