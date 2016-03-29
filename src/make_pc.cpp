#include <HAL/Camera/CameraDevice.h>
#include <SceneGraph/SceneGraph.h>
#include <calibu/Calibu.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <pangolin/pangolin.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[])
{
  // check arguments
  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " /path/to/cameras.xml 'scheme:///path/to/stereo/data'" << std::endl;
    return 1;
  }

  // read cameras.xml, get baseline
  std::shared_ptr<calibu::Rig<double>> rig =
    calibu::ReadXmlRig(argv[1]);
  float focal_length = rig->cameras_[1]->GetParams()[0];

  // open image data
  hal::Camera camera(argv[2]);
  if (camera.NumChannels() != 2) {
    std::cerr << "Expecting video channel and depth channel." << std::endl;
    return 1;
  }
  const unsigned w = camera.Width();
  const unsigned h = camera.Height();

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
  for (unsigned frame_number = 0; !pangolin::ShouldQuit(); frame_number++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.Capture(*imgs)) {
      const unsigned char* v_img = (*imgs)[0]->data();
      const unsigned short* d_img = (unsigned short*) (*imgs)[1]->data();

      video_view.SetImage(v_img, w, h);
      depth_view.SetImage(d_img, w, h, GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_SHORT);

      cloud.points.resize(cloud.width*(cloud.height+1));
      for (unsigned i=0; i<h*w; i++) {
        if (d_img[i] == 0) {
          cloud.points[i+cloud.height*h*w].x =
            cloud.points[i+cloud.height*h*w].y =
            cloud.points[i+cloud.height*h*w].z =
            std::numeric_limits<float>::quiet_NaN();
          continue;
        }
        // unproject depth
        pixel(0) = (float) (i/w);
        pixel(1) = (float) (i%w);
        float depth = d_img[i] / 10000.0;
        depth *= std::sqrt(focal_length*focal_length + (i/w - h/2)*(i/w - h/2)
            + (i%w - w/2)*(i%w - w/2));
        depth /= 10.;
        point = -depth * rig->cameras_[1]->Unproject(pixel);

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
