// Copyright 2017 John Stechschulte
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <numeric>

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <point_cloud_file> <frame> [frame ... ]" <<
      std::endl;
    return 1;
  }

  // open point cloud file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1) {
    std::cerr << "Error reading file " << argv[1] << std::endl;
    exit(1);
  }
  std::cerr << "Read " << cloud->height << " clouds of " << cloud->width <<
    " points." << std::endl;
  uint32_t cloud_n = 0;

  std::vector<int> indices(cloud->width);
  pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string basename = argv[1];
  basename.erase(basename.find_last_of('.'), std::string::npos);
  std::ostringstream filename;

  for (uint32_t i=2; i < argc; i++) {
    int fn = atoi(argv[i]);
    std::iota(indices.begin(), indices.end(), fn*cloud->width);
    subcloud->clear();
    pcl::copyPointCloud(*cloud, indices, *subcloud);
    filename << basename << "_" << fn << ".pcd";
    pcl::io::savePCDFileBinary(filename.str(), *subcloud);
    filename.str("");
    filename.clear();
  }

  return 0;
}
