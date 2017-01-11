#include <iostream>
#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "icp.h"

#define W 1024
#define H 768

using namespace pcl;
const float epsilon = 1e-4;

int main(int argc, char* argv[]) {
  std::vector<std::vector<float>> ref_pts = {
     {8.1472, 0.9754, 1.5761},
     {9.0579, 2.7850, 9.7059},
     {1.2699, 5.4688, 9.5717},
     {9.1338, 9.5751, 4.8538},
     {6.3236, 9.6489, 8.0028}
  };
  std::vector<std::vector<float>> src_pts = {
     {-10.7360,   1.1373,  -6.2434},
     {-23.9008,   2.6802,   4.0082},
     {-13.6096,  -0.6760,  16.4309},
     {-19.2731, -13.2004,   1.7711},
     {-19.7536, -10.6983,   9.8199}
  };
  /*
  std::vector<std::vector<float>> src_pts = {
    { 8.8294,  3.6256,  2.4910},
    {10.8155,  4.5095, 10.5825},
    { 2.8133,  5.8770, 11.9861},
    { 9.1199, 11.7824,  6.8562},
    { 6.8916, 10.9934, 10.3535}
  };
  */
  std::vector<int> match_i = {0, 1, 2, 3, 4};

  PointCloud<PointXYZ> ref_cloud;
  PointCloud<PointXYZ> src_cloud;

  ref_cloud.width = 5;
  ref_cloud.height = 1;
  ref_cloud.is_dense = true;
  ref_cloud.points.resize(ref_cloud.width*ref_cloud.height);

  src_cloud.width = 5;
  src_cloud.height = 1;
  src_cloud.is_dense = true;
  src_cloud.points.resize(src_cloud.width*src_cloud.height);

  for (size_t i=0; i<ref_cloud.points.size(); i++) {
    ref_cloud.points[i].x = ref_pts[i][0];
    ref_cloud.points[i].y = ref_pts[i][1];
    ref_cloud.points[i].z = ref_pts[i][2];
    src_cloud.points[i].x = src_pts[i][0];
    src_cloud.points[i].y = src_pts[i][1];
    src_cloud.points[i].z = src_pts[i][2];
  }

  Eigen::Matrix<float, 4, 4> m = localize(ref_cloud.makeShared(), src_cloud.makeShared(),
      match_i);

  bool passed=true;
  for (size_t i=0; i<src_cloud.points.size(); i++) {
    passed = passed && ((ref_cloud.points[i].getVector4fMap() -
          m*src_cloud.points[i].getVector4fMap()).norm() < epsilon);
  }
  if (!passed) {
    std::cout << ":-( localize failed" << std::endl;
  } else {
    std::cout << ":-D localize passed" << std::endl;
  }

  if (argc < 2) return 0;

  int f1 = 0;
  int f2 = 1;

  if (argc > 2) {
    f1 = atoi(argv[2]);
  }

  if (argc > 3) {
    f2 = atoi(argv[3]);
  } else {
    f2 = f1+1;
  }

  // open point cloud file
  pcl::PointCloud<pcl::PointXYZ>::Ptr real_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *real_cloud) == -1) {
    std::cout << "Error reading file " << argv[1] << std::endl;
    exit(1);
  }
  std::cout << "Read " << real_cloud->height << " clouds of " << real_cloud->width <<
    " points." << std::endl;

  std::vector<int> cloud_i(real_cloud->width);
  for (size_t i=0; i<cloud_i.size(); i++) {
    cloud_i[i] = i + f1*cloud_i.size();
  }
  pcl::PointCloud<pcl::PointXYZ> real_ref_cloud(*real_cloud, cloud_i);
  for (size_t i=0; i<cloud_i.size(); i++) {
    cloud_i[i] = i+f2*cloud_i.size();
  }
  pcl::PointCloud<pcl::PointXYZ> real_src_cloud(*real_cloud, cloud_i);

  Eigen::Matrix<float, 4, 4> Trs = Eigen::Matrix<float, 4, 4>::Identity();

  std::vector<bool> matched;
  ICP(real_ref_cloud.makeShared(), real_src_cloud.makeShared(), Trs, 10.0, &matched);
  Eigen::Matrix<float, 4, 4> Tsr = Eigen::Matrix<float, 4, 4>::Identity();
  Tsr.block(0, 0, 3, 3) = Trs.block(0, 0, 3, 3).transpose();
  Tsr.block(0, 3, 3, 1) = -Tsr.block(0, 0, 3, 3) * Trs.block(0, 3, 3, 1);
  std::cout << Tsr.block(0, 0, 3, 3) << "  ";
  std::cout << Tsr.block(0, 3, 3, 1).transpose() << std::endl;

  pcl::PointCloud<pcl::PointXYZ> out_cloud;
  out_cloud.height = 1;
  out_cloud.width = 307200 * 3;
  out_cloud.is_dense = false;
  out_cloud.resize(307200 * 3);

  for (size_t i=0; i<307200; i++) {
    out_cloud.points[0*307200 + i].getVector4fMap() =
      real_ref_cloud.points[i].getVector4fMap();
    out_cloud.points[1*307200 + i].getVector4fMap() = Trs *
      real_src_cloud.points[i].getVector4fMap();
    out_cloud.points[2*307200 + i].getVector4fMap() =
      real_src_cloud.points[i].getVector4fMap();
  }

  //pcl::io::savePCDFileBinary("registeredclouds.pcd", out_cloud);

  // start pangolin
  pangolin::CreateWindowAndBind("Main", W, H);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  pangolin::View& base_view = pangolin::DisplayBase();

  // create 3d view
  pangolin::OpenGlRenderState stacks3d(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 1E-3, 10*1000),
      pangolin::ModelViewLookAt(10, 0, 16, 0, 0, 0, pangolin::AxisNegZ)
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
  // reference cloud: red
  SceneGraph::GLCachedPrimitives glPCref(GL_POINTS, SceneGraph::GLColor(1.0f, 0.0f, 0.0f));
  // transformed source cloud: blue
  SceneGraph::GLCachedPrimitives glPCsrc(GL_POINTS, SceneGraph::GLColor(0.0f, 0.0f, 1.0f));
  // unmatched source cloud: light blue
  SceneGraph::GLCachedPrimitives glPCsru(GL_POINTS, SceneGraph::GLColor(0.5f, 0.5f, 1.0f));
  // original source cloud: dark gray
  SceneGraph::GLCachedPrimitives glPCsrr(GL_POINTS, SceneGraph::GLColor(0.3f, 0.3f, 0.3f));
  glGraph.AddChild(&glPCref);
  glGraph.AddChild(&glPCsrc);
  glGraph.AddChild(&glPCsru);
  glGraph.AddChild(&glPCsrr);

  bool draw = true;

  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    if (draw) {
      glPCref.Clear();
      glPCsrc.Clear();
      glPCsru.Clear();
      glPCsrr.Clear();
      for (uint32_t i = 0; i < 307200; i++){
        glPCref.AddVertex(Eigen::Vector3d(out_cloud.points[0*307200 + i].x,
              out_cloud.points[0*307200 + i].y, out_cloud.points[0*307200 +
              i].z));
        if (matched[i]) {
          glPCsrc.AddVertex(Eigen::Vector3d(out_cloud.points[1*307200 + i].x,
                out_cloud.points[1*307200 + i].y, out_cloud.points[1*307200 +
                i].z));
        } else {
          glPCsru.AddVertex(Eigen::Vector3d(out_cloud.points[1*307200 + i].x,
                out_cloud.points[1*307200 + i].y, out_cloud.points[1*307200 +
                i].z));
        }
        glPCsrr.AddVertex(Eigen::Vector3d(out_cloud.points[2*307200 + i].x,
              out_cloud.points[2*307200 + i].y, out_cloud.points[2*307200 +
              i].z));
      }
      draw = false;
    }

    pangolin::FinishFrame();
  }
}
