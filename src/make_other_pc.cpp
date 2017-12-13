// Copyright 2017 John Stechschulte
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <CImg.h>
#include <cmath>
#include <iomanip>
#include <iostream>

int main(int argc, char *argv[]) {
  float fx = 525.0;
  float fy = 525.0;
  float cx = 319.5;
  float cy = 239.5;

  for (int i = 1; i < argc; i++) {
    CImg<uint16_t> depth_image(argv[i]);
    std::cout << "W: " << depth_image.width() << std::endl;
    std::cout << "H: " << depth_image.height() << std::endl;
    uint16_t v1 = depth_image(0, 0, 0, 0);
    uint16_t v2 = depth_image(20, 30, 0, 0);
    std::cout << v1 << " " << v2 << std::endl;
    break;
  }
  // open image data
  return 0;
}
