#include "kdtree.h"
#include <iostream>
#include <iomanip>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cstdlib>
#include <time.h>
#include <chrono>
#include <mach/mach.h>

#define FW 11

const float num_searches = 50;
const float num_trees = 10;
const int max_lg_points = 24;

size_t get_mem_usage() {
  task_t targetTask = mach_task_self();
  struct task_basic_info ti;
  mach_msg_type_number_t count = TASK_BASIC_INFO_64_COUNT;

  kern_return_t kr = task_info(targetTask, TASK_BASIC_INFO_64,
      (task_info_t) &ti, &count);
  if (kr != KERN_SUCCESS) {
    printf("Kernel returned error during memory usage query");
    return -1;
  }

  return ti.resident_size;
}

int main(int argc, char* argv[]) {
  unsigned seed = time(NULL);
  if (argc > 1) {
    seed = strtol(argv[1], NULL, 10);
  }
  std::cout << "Seed: " << seed << std::endl;
  srand(seed);

  KDTree kdtree;
  pcl::PointCloud<pcl::PointXYZ> cloud;

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

  std::cout << "Tree building test" << std::endl;
  kdtree.setInputCloud(cloud.makeShared());

  if (!(kdtree.point.x ==  8 && kdtree.point.y ==  7 && kdtree.point.z ==  5)) std::cout << "build_tree failed root" << std::endl;
  if (!(kdtree.l->point.x ==  3 && kdtree.l->point.y == 12 && kdtree.l->point.z == 14)) std::cout << "build_tree failed l" << std::endl;
  if (!(kdtree.r->point.x == 12 && kdtree.r->point.y ==  3 && kdtree.r->point.z == 10)) std::cout << "build_tree failed r" << std::endl;
  if (!(kdtree.l->l->point.x ==  7 && kdtree.l->l->point.y ==  8 && kdtree.l->l->point.z ==  7)) std::cout << "build_tree failed ll" << std::endl;
  if (!(kdtree.l->r->point.x ==  2 && kdtree.l->r->point.y == 13 && kdtree.l->r->point.z ==  9)) std::cout << "build_tree failed lr" << std::endl;
  if (!(kdtree.r->l->point.x == 14 && kdtree.r->l->point.y ==  1 && kdtree.r->l->point.z == 11)) std::cout << "build_tree failed rl" << std::endl;
  if (!(kdtree.r->r->point.x == 11 && kdtree.r->r->point.y ==  4 && kdtree.r->r->point.z ==  6)) std::cout << "build_tree failed rr" << std::endl;
  if (!(kdtree.l->l->l->point.x ==  6 && kdtree.l->l->l->point.y ==  9 && kdtree.l->l->l->point.z ==  1)) std::cout << "build_tree failed lll" << std::endl;
  if (!(kdtree.l->l->r->point.x ==  4 && kdtree.l->l->r->point.y == 11 && kdtree.l->l->r->point.z ==  8)) std::cout << "build_tree failed llr" << std::endl;
  if (!(kdtree.l->r->l->point.x ==  1 && kdtree.l->r->l->point.y == 14 && kdtree.l->r->l->point.z ==  4)) std::cout << "build_tree failed lrl" << std::endl;
  if (!(kdtree.l->r->r->point.x ==  0 && kdtree.l->r->r->point.y == 15 && kdtree.l->r->r->point.z == 13)) std::cout << "build_tree failed lrr" << std::endl;
  if (!(kdtree.r->l->l->point.x == 13 && kdtree.r->l->l->point.y ==  2 && kdtree.r->l->l->point.z ==  0)) std::cout << "build_tree failed rll" << std::endl;
  if (!(kdtree.r->l->r->point.x == 15 && kdtree.r->l->r->point.y ==  0 && kdtree.r->l->r->point.z == 12)) std::cout << "build_tree failed rlr" << std::endl;
  if (!(kdtree.r->r->l->point.x ==  9 && kdtree.r->r->l->point.y ==  6 && kdtree.r->r->l->point.z ==  3)) std::cout << "build_tree failed rrl" << std::endl;
  if (!(kdtree.r->r->r->point.x == 10 && kdtree.r->r->r->point.y ==  5 && kdtree.r->r->r->point.z == 15)) std::cout << "build_tree failed rrr" << std::endl;
  if (!(kdtree.l->l->l->l->point.x ==  5 && kdtree.l->l->l->l->point.y == 10 && kdtree.l->l->l->l->point.z ==  2)) std::cout << "build_tree failed llll" << std::endl;

  int num_points = 100;
  cloud.resize(num_points);
  for (int i=0; i<num_points; i++) {
    cloud.points[i].x = (100. * rand())/RAND_MAX;
    cloud.points[i].y = (100. * rand())/RAND_MAX;
    cloud.points[i].z = (100. * rand())/RAND_MAX;
  }

  pcl::PointXYZ test_point;
  test_point.x = (100. * rand())/RAND_MAX;
  test_point.y = (100. * rand())/RAND_MAX;
  test_point.z = (100. * rand())/RAND_MAX;

  std::vector<int> nearest_i;
  std::vector<float> nearest_d;

  kdtree.setInputCloud(cloud.makeShared());
  kdtree.nearestKSearch(test_point, 1, nearest_i, nearest_d);

  for (int i=0; i<num_points; i++) {
    if (sqrt(dist2(test_point, cloud.points[i])) < nearest_d[0] &&
        i != nearest_i[0] ) {
      std::cout << "Nearest neighbor search failed" << std::endl;
      std::cout << sqrt(dist2(test_point, cloud.points[i])) << " " << nearest_d[0] <<
        std::endl << std::endl;
      std::cout << printpoint(test_point) << std::endl;
      std::cout << i << " " << nearest_i[0] << std::endl;
      std::cout << kdtree << std::endl;
      printpoints(cloud.points);
      break;
    }
  }

  // performance tests
  std::chrono::duration<uint64_t, std::micro> kdtree_build_time(0);
  std::chrono::duration<uint64_t, std::micro> kdtree_search_time(0);
  std::chrono::duration<uint64_t, std::micro> kdtree_ext_build_time(0);
  std::chrono::duration<uint64_t, std::micro> kdtree_ext_search_time(0);
  size_t kdtree_size;
  size_t kdtree_ext_size;
  auto start = std::chrono::high_resolution_clock::now();
  auto stop = std::chrono::high_resolution_clock::now();
  pcl::PointXYZ search_point;

  std::cout << "Performance tests" << std::endl;
  std::cout << std::setw(FW) << "lg n" <<
      " " << std::setw(FW) << "n lg n" <<
      " " << std::setw(FW) << "build" <<
      " " << std::setw(FW) << "ext" <<
      " " << std::setw(FW) << "search" <<
      " " << std::setw(FW) << "ext" <<
      " " << std::setw(FW) << "size" <<
      " " << std::setw(FW) << "ext" <<
      std::endl;

  for (int lg_points=4; lg_points<=max_lg_points; lg_points++) {
    cloud.resize(1<<lg_points);
    kdtree_build_time = std::chrono::microseconds::zero();
    kdtree_ext_build_time = std::chrono::microseconds::zero();
    kdtree_search_time = std::chrono::microseconds::zero();
    kdtree_ext_search_time = std::chrono::microseconds::zero();

    for (int tree=0; tree<num_trees; tree++) {
      for (int i=0; i<(1<<lg_points); i++) {
        cloud.points[i].x = (100.*rand())/RAND_MAX;
        cloud.points[i].y = (100.*rand())/RAND_MAX;
        cloud.points[i].z = (100.*rand())/RAND_MAX;
      }

      // my KDTree
      {
        KDTree kdtree_mine;
        start = std::chrono::high_resolution_clock::now();
        kdtree_mine.setInputCloud(cloud.makeShared());
        stop = std::chrono::high_resolution_clock::now();
        kdtree_build_time +=
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        kdtree_size = get_mem_usage();

        start = std::chrono::high_resolution_clock::now();
        for (int i=0; i<num_searches; i++) {
          search_point.x = (100.*rand())/RAND_MAX;
          search_point.y = (100.*rand())/RAND_MAX;
          search_point.z = (100.*rand())/RAND_MAX;
          kdtree_mine.nearestKSearch(search_point, 1, nearest_i, nearest_d);
        }
        stop = std::chrono::high_resolution_clock::now();
        kdtree_search_time +=
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      }

      // pcl KDTree
      {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_ext;
        start = std::chrono::high_resolution_clock::now();
        kdtree_ext.setInputCloud(cloud.makeShared());
        stop = std::chrono::high_resolution_clock::now();
        kdtree_ext_build_time +=
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        kdtree_ext_size = get_mem_usage();

        start = std::chrono::high_resolution_clock::now();
        for (int i=0; i<1000; i++) {
          search_point.x = (100.*rand())/RAND_MAX;
          search_point.y = (100.*rand())/RAND_MAX;
          search_point.z = (100.*rand())/RAND_MAX;
          kdtree_ext.nearestKSearch(search_point, 1, nearest_i, nearest_d);
        }
        stop = std::chrono::high_resolution_clock::now();
        kdtree_ext_search_time +=
          std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
      }
    }

    std::cout << std::setw(FW) << lg_points <<
      " " << std::setw(FW) << ((int64_t)lg_points)*(1<<lg_points) <<
      " " << std::setw(FW) << kdtree_build_time.count()/num_trees <<
      " " << std::setw(FW) << kdtree_ext_build_time.count()/num_trees <<
      " " << std::setw(FW) << kdtree_search_time.count()/(num_searches*num_trees) <<
      " " << std::setw(FW) << kdtree_ext_search_time.count()/(num_searches*num_trees) <<
      " " << std::setw(FW) << kdtree_size <<
      " " << std::setw(FW) << kdtree_ext_size <<
      std::endl;
  }

}
