// Copyright 2017 John Stechschulte
#include <HAL/Camera/CameraDevice.h>
#include <SceneGraph/SceneGraph.h>
#include <calibu/Calibu.h>
#include <pangolin/pangolin.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>
#include <iomanip>
#include <iostream>

int main(int argc, char *argv[]) {
  // check arguments
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " /path/to/cameras.xml 'scheme:///path/to/stereo/data'" << std::endl;
    return 1;
  }

  // read cameras.xml, get baseline
  std::shared_ptr<calibu::Rig<double>> rig =
    calibu::ReadXmlRig(argv[1]);
  float focal_length = rig->cameras_[1]->GetParams()[0];
  float u0 = rig->cameras_[1]->GetParams()[2];
  float v0 = rig->cameras_[1]->GetParams()[3];

  // open image data
  hal::Camera camera(argv[2]);
  if (camera.NumChannels() != 2) {
    std::cerr << "Expecting video channel and depth channel." << std::endl;
    return 1;
  }
  const uint32_t w = camera.Width();
  const uint32_t h = camera.Height();

  // start Pangolin, create displays
  pangolin::CreateWindowAndBind("Main", 2*w, h);
  pangolin::View& base_view = pangolin::DisplayBase();

  // video, depth views
  SceneGraph::ImageView video_view(true, false);
  SceneGraph::ImageView depth_view(true, false);

  video_view.SetBounds(0.0, 1.0, 0.0, 0.5);
  depth_view.SetBounds(0.0, 1.0, 0.5, 1.0);

  base_view.AddDisplay(video_view);
  base_view.AddDisplay(depth_view);

  // memory stuff
  std::shared_ptr<hal::ImageArray> imgs = hal::ImageArray::Create();

  Eigen::Vector2d pixel;
  Eigen::Vector3d point;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = w*h;
  cloud.height = 0;
  cloud.is_dense = false;

  // main loop
  for (uint32_t frame_number = 0; !pangolin::ShouldQuit(); frame_number++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.Capture(*imgs)) {
      const uint8_t* v_img = (*imgs)[0]->data();
      const uint16_t* d_img = reinterpret_cast<const uint16_t*>((*imgs)[1]->data());

      video_view.SetImage(v_img, w, h);
      depth_view.SetImage(d_img, w, h, GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_SHORT);

      cloud.points.resize(cloud.width*(cloud.height+1));
      for (uint32_t i=0; i < h*w; i++) {
        if (d_img[i] == 0) {
          cloud.points[i+cloud.height*h*w].x =
            cloud.points[i+cloud.height*h*w].y =
            cloud.points[i+cloud.height*h*w].z =
            std::numeric_limits<float>::quiet_NaN();
          continue;
        }
        // unproject depth
        pixel(0) = static_cast<float>(i%w);
        pixel(1) = static_cast<float>(i/w);
        float depth = d_img[i] / 10000.0;
        depth *= std::sqrt(focal_length*focal_length +
            (pixel(0) - u0)*(pixel(0) - u0) +
            (pixel(1) - v0)*(pixel(1) - v0))/focal_length;
        point = depth * rig->cameras_[1]->Unproject(pixel);

        // append to point cloud
        cloud.points[i+cloud.height*h*w].x = point(0);
        cloud.points[i+cloud.height*h*w].y = point(1);
        cloud.points[i+cloud.height*h*w].z = point(2);
      }
      cloud.height++;
    }
    pangolin::FinishFrame();
  }
  pcl::io::savePCDFileBinary("pointclouds.pcd", cloud);

  return 0;
}
