#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>

#define W 1024
#define H 768

void filter_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.5);
  sor.filter(*cloud);
  return;
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " [-f] <output_cloud_file> <point_cloud_files>" << std::endl;
    exit(1);
  }

  bool filter = false;
  if (argv[1][0] == '-' && argv[1][1] == 'f') {
    filter = true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i=2+filter; i<argc; i++) {
    // open point cloud file
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i], *in_cloud) == -1) {
      std::cerr << "Error reading file " << argv[i] << std::endl;
      exit(1);
    }
    std::cerr << "Read " << argv[i] << " containing " << in_cloud->height << 
      " clouds of " << in_cloud->width << " points." << std::endl;
    *cloud += *in_cloud;
  }
  std::cerr << cloud->width << " points total" << std::endl;

  if (filter) {
    filter_pointcloud(cloud);
  }

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (argv[1+filter], *cloud, true);
}
