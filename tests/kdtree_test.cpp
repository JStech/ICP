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

  kdtree.setInputCloud(cloud.makeShared());

  if (!(kdtree.point.x ==  8 && kdtree.point.y ==  7 && kdtree.point.z ==  5)) std::cerr << "build_tree failed root" << std::endl;
  if (!(kdtree.l->point.x ==  3 && kdtree.l->point.y == 12 && kdtree.l->point.z == 14)) std::cerr << "build_tree failed l" << std::endl;
  if (!(kdtree.r->point.x == 12 && kdtree.r->point.y ==  3 && kdtree.r->point.z == 10)) std::cerr << "build_tree failed r" << std::endl;
  if (!(kdtree.l->l->point.x ==  7 && kdtree.l->l->point.y ==  8 && kdtree.l->l->point.z ==  7)) std::cerr << "build_tree failed ll" << std::endl;
  if (!(kdtree.l->r->point.x ==  2 && kdtree.l->r->point.y == 13 && kdtree.l->r->point.z ==  9)) std::cerr << "build_tree failed lr" << std::endl;
  if (!(kdtree.r->l->point.x == 14 && kdtree.r->l->point.y ==  1 && kdtree.r->l->point.z == 11)) std::cerr << "build_tree failed rl" << std::endl;
  if (!(kdtree.r->r->point.x == 11 && kdtree.r->r->point.y ==  4 && kdtree.r->r->point.z ==  6)) std::cerr << "build_tree failed rr" << std::endl;
  if (!(kdtree.l->l->l->point.x ==  6 && kdtree.l->l->l->point.y ==  9 && kdtree.l->l->l->point.z ==  1)) std::cerr << "build_tree failed lll" << std::endl;
  if (!(kdtree.l->l->r->point.x ==  4 && kdtree.l->l->r->point.y == 11 && kdtree.l->l->r->point.z ==  8)) std::cerr << "build_tree failed llr" << std::endl;
  if (!(kdtree.l->r->l->point.x ==  1 && kdtree.l->r->l->point.y == 14 && kdtree.l->r->l->point.z ==  4)) std::cerr << "build_tree failed lrl" << std::endl;
  if (!(kdtree.l->r->r->point.x ==  0 && kdtree.l->r->r->point.y == 15 && kdtree.l->r->r->point.z == 13)) std::cerr << "build_tree failed lrr" << std::endl;
  if (!(kdtree.r->l->l->point.x == 13 && kdtree.r->l->l->point.y ==  2 && kdtree.r->l->l->point.z ==  0)) std::cerr << "build_tree failed rll" << std::endl;
  if (!(kdtree.r->l->r->point.x == 15 && kdtree.r->l->r->point.y ==  0 && kdtree.r->l->r->point.z == 12)) std::cerr << "build_tree failed rlr" << std::endl;
  if (!(kdtree.r->r->l->point.x ==  9 && kdtree.r->r->l->point.y ==  6 && kdtree.r->r->l->point.z ==  3)) std::cerr << "build_tree failed rrl" << std::endl;
  if (!(kdtree.r->r->r->point.x == 10 && kdtree.r->r->r->point.y ==  5 && kdtree.r->r->r->point.z == 15)) std::cerr << "build_tree failed rrr" << std::endl;
  if (!(kdtree.l->l->l->l->point.x ==  5 && kdtree.l->l->l->l->point.y == 10 && kdtree.l->l->l->l->point.z ==  2)) std::cerr << "build_tree failed llll" << std::endl;

  int n = strtol(argv[1], NULL, 10);
  cloud.resize(n);

  for (int i=0; i<n; i++) {
    cloud.points[i].x = floor((100. * rand())/RAND_MAX);
    cloud.points[i].y = floor((100. * rand())/RAND_MAX);
    cloud.points[i].z = floor((100. * rand())/RAND_MAX);
  }
  printpoints(cloud.points);
  std::cout << std::endl;

  kdtree.setInputCloud(cloud.makeShared());

  std::cout << kdtree << std::endl;
}
