#include <HAL/Camera/CameraDevice.h>
#include <SceneGraph/SceneGraph.h>
#include <calibu/Calibu.h>
#include <elas/elas.h>
#include <iostream>
#include <pangolin/pangolin.h>

int main(int argc, char *argv[])
{
  // check arguments
  if (argc != 4) {
    std::cout << "Usage: " << argv[0] << " /path/to/cameras.xml 'scheme:///path/to/stereo/data' <max depth>" << std::endl;
    return 1;
  }

  float max_d = atof(argv[3]);

  // read cameras.xml, get baseline
  std::shared_ptr<calibu::Rig<double>> rig =
    calibu::ReadXmlRig(argv[1]);
  double baseline = (rig->cameras_[0]->Pose().translation() - rig->cameras_[1]->Pose().translation()).norm();
  double focal_length = rig->cameras_[0]->GetParams()[0];

  // open stereo data
  hal::Camera camera(argv[2]);
  if (camera.NumChannels() < 2) {
    std::cerr << "Must run on stereo data." << std::endl;
    return 1;
  }
  const unsigned w = camera.Width();
  const unsigned h = camera.Height();

  bool true_depth = false;
  if (camera.NumChannels() == 3) {
    true_depth = true;
  }

  // instantiate ELAS
  Elas::parameters eparams;
  eparams.postprocess_only_left = true;
  Elas elas(eparams);

  // start Pangolin, create displays
  pangolin::CreateWindowAndBind("Main", 2*w, 2*h);
  pangolin::View& base_view = pangolin::DisplayBase();

  // L, R, disparity views
  SceneGraph::ImageView l_view(true, false);
  SceneGraph::ImageView r_view(true, false);
  SceneGraph::ImageView dl_view(true, false);
  SceneGraph::ImageView dr_view(true, false);

  l_view.SetBounds(0.5, 1.0, 0.0, 0.5);
  r_view.SetBounds(0.5, 1.0, 0.5, 1.0);
  dl_view.SetBounds(0.0, 0.5, 0.0, 0.5);
  dr_view.SetBounds(0.0, 0.5, 0.5, 1.0);

  base_view.AddDisplay(l_view);
  base_view.AddDisplay(r_view);
  base_view.AddDisplay(dl_view);
  base_view.AddDisplay(dr_view);

  // memory stuff
  std::shared_ptr<hal::ImageArray> imgs = hal::ImageArray::Create();

  // allocate memory for disparity images
  const uint32_t dims[3] = {w, h, w}; // bytes per line = width
  float* D1_data = (float*)malloc(w*h*sizeof(float));
  float* D2_data = (float*)malloc(w*h*sizeof(float));

  float max_e2 = 0;

  // main loop
  for (unsigned frame_number = 0; !pangolin::ShouldQuit(); frame_number++) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (camera.Capture(*imgs)) {
      float* d_img;
      const unsigned char* l_img = (*imgs)[0]->data();
      const unsigned char* r_img = (*imgs)[1]->data();
      if (true_depth) {
        d_img = (float*) (*imgs)[2]->data();
      }
      elas.process(l_img, r_img, D1_data, D2_data, (const int *) dims);

      for (uint i=0; i<w*h; i++) {
        D1_data[i] = baseline * focal_length / D1_data[i];
        if (true_depth) {
          float e2 = (D1_data[i] - d_img[i])*(D1_data[i] - d_img[i]);
          if (e2 > max_e2) {
            max_e2 = e2;
          }
          d_img[i] /= max_d;
        }
        D1_data[i] /= max_d;
      }

      l_view.SetImage(l_img, w, h, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
      r_view.SetImage(r_img, w, h, GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
      dl_view.SetImage(D1_data, w, h, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
      if (true_depth) {
        dr_view.SetImage(d_img, w, h, GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
      }
    }
    pangolin::FinishFrame();
  }
  std::cout << max_e2 << std::endl;

  return 0;
}
