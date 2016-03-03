#include <HAL/Camera/CameraDevice.h>
#include <SceneGraph/SceneGraph.h>
#include <calibu/Calibu.h>
#include <iostream>
#include <pangolin/pangolin.h>

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

  //FILE* pcf = fopen("point_cloud.bin", "wb");

  Eigen::Vector2d pixel;
  Eigen::Vector3d point;

  // main loop
  for (unsigned frame_number = 0; !pangolin::ShouldQuit(); frame_number++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.Capture(*imgs)) {
      const unsigned char* v_img = (*imgs)[0]->data();
      const float* d_img = (float*) (*imgs)[1]->data();

      video_view.SetImage(v_img, w, h);
      depth_view.SetImage(d_img, w, h, GL_LUMINANCE, GL_LUMINANCE, GL_UNSIGNED_SHORT);

      /*
      float p[3];
      for (unsigned i=0; i<h*w; i++) {
        if (D1_data[i] < 0) continue;
        // unproject depth
        pixel(0) = (float) (i/w);
        pixel(1) = (float) (i%w);
        point = D1_data[i] * rig->cameras_[0]->Unproject(pixel);

        // append to point cloud
        // TODO: do this more efficiently
        p[0] = point(0);
        p[1] = point(1);
        p[2] = point(2);
        fwrite(p, sizeof(float), 3, pcf);
      }
      p[0] = p[1] = p[2] = 0;
      fwrite(p, sizeof(float), 3, pcf);
      */
    }
    pangolin::FinishFrame();
  }
  //fclose(pcf);

  return 0;
}
