#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define W 1024
#define H 768

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <point_cloud_file>" << std::endl;
    exit(1);
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

  // start pangolin
  pangolin::CreateWindowAndBind("Main", W, H);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  pangolin::View& base_view = pangolin::DisplayBase();

  // create 3d view
  pangolin::OpenGlRenderState stacks3d(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 1E-3, 10*1000),
      pangolin::ModelViewLookAt(0, -.2, 0, 0, 0, 1, pangolin::AxisNegY)
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

    if (step && cloud_n < cloud->height) {
      glPC.Clear();
      for (uint32_t i = cloud_n*cloud->width; i<(cloud_n+1)*cloud->width; i++){
        glPC.AddVertex(Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z));
      }
      cloud_n++;
      step = false;
    }

    pangolin::FinishFrame();
  }
}
