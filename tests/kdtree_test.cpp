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

  cloud.resize(16);
  cloud.points[ 0].x =  0; cloud.points[ 0].y = 15; cloud.points[ 0].z = 13;
  cloud.points[ 1].x =  1; cloud.points[ 1].y = 14; cloud.points[ 1].z =  4;
  cloud.points[ 2].x =  2; cloud.points[ 2].y = 13; cloud.points[ 2].z =  9;
  cloud.points[ 3].x =  3; cloud.points[ 3].y = 12; cloud.points[ 3].z = 14;
  cloud.points[ 4].x =  4; cloud.points[ 4].y = 11; cloud.points[ 4].z =  8;
  cloud.points[ 5].x =  5; cloud.points[ 5].y = 10; cloud.points[ 5].z =  2;
  cloud.points[ 6].x =  6; cloud.points[ 6].y =  9; cloud.points[ 6].z =  1;
  cloud.points[ 7].x =  7; cloud.points[ 7].y =  8; cloud.points[ 7].z =  7;
  cloud.points[ 8].x =  8; cloud.points[ 8].y =  7; cloud.points[ 8].z =  5;
  cloud.points[ 9].x =  9; cloud.points[ 9].y =  6; cloud.points[ 9].z =  3;
  cloud.points[10].x = 10; cloud.points[10].y =  5; cloud.points[10].z = 15;
  cloud.points[11].x = 11; cloud.points[11].y =  4; cloud.points[11].z =  6;
  cloud.points[12].x = 12; cloud.points[12].y =  3; cloud.points[12].z = 10;
  cloud.points[13].x = 13; cloud.points[13].y =  2; cloud.points[13].z =  0;
  cloud.points[14].x = 14; cloud.points[14].y =  1; cloud.points[14].z = 11;
  cloud.points[15].x = 15; cloud.points[15].y =  0; cloud.points[15].z = 12;

  // test select
  for (int tests=0; tests<1000; tests++) {
    int dim = rand()%3;
    int start = rand()%16;
    int end = rand()%16;
    while (end <= start) {
      start = rand()%16;
      end = rand()%16;
    }
    int k = rand()%(end-start) + start;
    KDTree::select(cloud.points, dim, start, end, k);
    bool pass = true;
    for (int i=start; i<k; i++) {
      pass = pass && (cloud.points[i].data[dim] < cloud.points[k].data[dim]);
    }
    for (int i=k+1; i<end; i++) {
      pass = pass && (cloud.points[k].data[dim] < cloud.points[i].data[dim]);
    }
    if (!pass) {
      std::cerr << "FAILED " << dim << " " << start << " " << end << " " << k
        << std::endl;
      break;
    }
  }

  printpoints(cloud.points);
  kdtree.setInputCloud(cloud.makeShared());

  if (!(cloud.points[ 0].x ==  5 && cloud.points[ 0].y == 10 && cloud.points[ 0].z ==  2)) std::cerr << "build_tree point selection failed  0" << std::endl;
  if (!(cloud.points[ 1].x ==  6 && cloud.points[ 1].y ==  9 && cloud.points[ 1].z ==  1)) std::cerr << "build_tree point selection failed  1" << std::endl;
  if (!(cloud.points[ 2].x ==  4 && cloud.points[ 2].y == 11 && cloud.points[ 2].z ==  8)) std::cerr << "build_tree point selection failed  2" << std::endl;
  if (!(cloud.points[ 3].x ==  7 && cloud.points[ 3].y ==  8 && cloud.points[ 3].z ==  7)) std::cerr << "build_tree point selection failed  3" << std::endl;
  if (!(cloud.points[ 4].x ==  1 && cloud.points[ 4].y == 14 && cloud.points[ 4].z ==  4)) std::cerr << "build_tree point selection failed  4" << std::endl;
  if (!(cloud.points[ 5].x ==  2 && cloud.points[ 5].y == 13 && cloud.points[ 5].z ==  9)) std::cerr << "build_tree point selection failed  5" << std::endl;
  if (!(cloud.points[ 6].x ==  0 && cloud.points[ 6].y == 15 && cloud.points[ 6].z == 13)) std::cerr << "build_tree point selection failed  6" << std::endl;
  if (!(cloud.points[ 7].x ==  3 && cloud.points[ 7].y == 12 && cloud.points[ 7].z == 14)) std::cerr << "build_tree point selection failed  7" << std::endl;
  if (!(cloud.points[ 8].x == 12 && cloud.points[ 8].y ==  3 && cloud.points[ 8].z == 10)) std::cerr << "build_tree point selection failed  8" << std::endl;
  if (!(cloud.points[ 9].x == 13 && cloud.points[ 9].y ==  2 && cloud.points[ 9].z ==  0)) std::cerr << "build_tree point selection failed  9" << std::endl;
  if (!(cloud.points[10].x == 14 && cloud.points[10].y ==  1 && cloud.points[10].z == 11)) std::cerr << "build_tree point selection failed 10" << std::endl;
  if (!(cloud.points[11].x == 15 && cloud.points[11].y ==  0 && cloud.points[11].z == 12)) std::cerr << "build_tree point selection failed 11" << std::endl;
  if (!(cloud.points[12].x ==  8 && cloud.points[12].y ==  7 && cloud.points[12].z ==  5)) std::cerr << "build_tree point selection failed 12" << std::endl;
  if (!(cloud.points[13].x ==  9 && cloud.points[13].y ==  6 && cloud.points[13].z ==  3)) std::cerr << "build_tree point selection failed 13" << std::endl;
  if (!(cloud.points[14].x == 10 && cloud.points[14].y ==  5 && cloud.points[14].z == 15)) std::cerr << "build_tree point selection failed 14" << std::endl;
  if (!(cloud.points[15].x == 11 && cloud.points[15].y ==  4 && cloud.points[15].z ==  6)) std::cerr << "build_tree point selection failed 15" << std::endl;
  std::cerr << std::endl;
  printpoints(cloud.points);
  std::cerr << std::endl;
  return 0;

  int n = strtol(argv[1], NULL, 10);
  cloud.resize(n);

  for (int i=0; i<n; i++) {
    cloud.points[i].x = floor((100. * rand())/RAND_MAX);
    cloud.points[i].y = floor((100. * rand())/RAND_MAX);
    cloud.points[i].z = floor((100. * rand())/RAND_MAX);
    std::cout << std::setw(4) << cloud.points[i].x << " " <<
      std::setw(4) << cloud.points[i].y << " " <<
      std::setw(4) << cloud.points[i].z << std::endl;
  }

  kdtree.setInputCloud(cloud.makeShared());

  std::cout << kdtree << std::endl;
}
