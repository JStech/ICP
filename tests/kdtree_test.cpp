#include "kdtree.h"
#include <iostream>
#include <iomanip>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cstdlib>

int main(int argc, char* argv[]) {
  KDTree kdtree;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <n>" << std::endl;
    return 1;
  }

  int n = strtol(argv[1], NULL, 10);
  cloud.resize(n);

  for (int i=0; i<n; i++) {
    cloud.points[i].x = ((float) rand())/RAND_MAX;
    cloud.points[i].y = ((float) rand())/RAND_MAX;
    cloud.points[i].z = ((float) rand())/RAND_MAX;
    std::cout << std::setw(12) << cloud.points[i].x << " " <<
      std::setw(12) << cloud.points[i].y << " " <<
      std::setw(12) << cloud.points[i].z << std::endl;
  }

  kdtree.setInputCloud(cloud.makeShared());

  std::cout << kdtree << std::endl;
}
