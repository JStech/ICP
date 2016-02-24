#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <iostream>

#define W 1024
#define H 768

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
    exit(1);
  }

  // open point cloud file
  FILE* pcf = fopen(argv[1], "rb");

  float point[3];
  int rv;

  // start pangolin
  pangolin::CreateWindowAndBind("Main", W, H);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  pangolin::View& base_view = pangolin::DisplayBase();

  // create 3d view
  pangolin::OpenGlRenderState stacks3d(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 1E-3, 10*1000),
      pangolin::ModelViewLookAt(-10, 0, -16, 0, 0, 0, pangolin::AxisNegZ)
      );
  SceneGraph::GLSceneGraph glGraph;
  pangolin::View threeD_view;

  SceneGraph::GLGrid glGrid(150, 1);
  glGraph.AddChild(&glGrid);
  threeD_view.SetBounds(0.0, 1.0, 0.0, 1.0)
    .SetHandler(new SceneGraph::HandlerSceneGraph(glGraph, stacks3d))
    .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));
  base_view.AddDisplay(threeD_view);

  // Reset background color to black.
  glClearColor(0, 0, 0, 1);

  // Point cloud GL object
  SceneGraph::GLCachedPrimitives glPC(GL_POINTS, SceneGraph::GLColor(1.0f,
        0.7f, 0.2f));
  glGraph.AddChild(&glPC);

  bool step = true;
  pangolin::RegisterKeyPressCallback(' ', [&]() {step = true;});

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    if (step) {
      glPC.Clear();
      rv = fread(point, sizeof(float), 3, pcf);
      uint32_t num_points = 0;
      while (rv==3 && (point[0] != 0. || point[1] != 0. || point[2] != 0.)) {
        glPC.AddVertex(Eigen::Vector3d(point[0], point[1], point[2]));
        num_points++;
        rv = fread(point, sizeof(float), 3, pcf);
      }
      if (num_points == 0) break;
      step = false;
    }

    pangolin::FinishFrame();
  }

  fclose(pcf);
}
