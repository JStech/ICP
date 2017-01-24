#include <iostream>
#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "icp.h"

#define W 1024
#define H 768
#define COMP_N (1<<16)

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
    passed = passed && ((ref_cloud.points[i].getVector3fMap() -
          (m.topLeftCorner(3,3)*src_cloud.points[i].getVector3fMap() +
           m.topRightCorner(3,1))).norm() < epsilon);
  }
  if (!passed) {
    std::cout << ":-( localize failed" << std::endl;
  } else {
    std::cout << ":-D localize passed" << std::endl;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_comp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_comp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  in_comp_cloud->points.resize(COMP_N);
  for (size_t i=0; i<COMP_N; i++) {
    in_comp_cloud->points[i].x = rand() / (RAND_MAX + 1.0f);
    in_comp_cloud->points[i].y = rand() / (RAND_MAX + 1.0f);
    in_comp_cloud->points[i].z = rand() / (RAND_MAX + 1.0f);
    in_comp_cloud->points[i].r = rand()%256;
    in_comp_cloud->points[i].g = rand()%256;
    in_comp_cloud->points[i].b = rand()%256;
  }

  downsample_cloud(0.05, in_comp_cloud, out_comp_cloud);

  std::cout << "Compression:" << std::endl;
  std::cout << "  input points,  " << COMP_N << std::endl;
  std::cout << "  output points, " << out_comp_cloud->size() << std::endl;

  if (argc < 3) return 0;

  // open point cloud file
  pcl::PointCloud<pcl::PointXYZ>::Ptr real_ref_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr real_src_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *real_ref_cloud) == -1) {
    std::cout << "Error reading file " << argv[1] << std::endl;
    exit(1);
  }
  std::cout << "Read " << real_ref_cloud->height << " clouds of " << real_ref_cloud->width <<
    " points." << std::endl;
  int ref_n = real_ref_cloud->width;

  /*
  Eigen::Matrix<float, 4, 4> T_init;
  T_init <<  1.0010,  -0.0041,   0.0082,   0.03,
             0.0041,   1.0010,  -0.0041,   0.01,
            -0.0082,   0.0041,   1.0010,   0.05,
                  0,        0,        0,      1;
  for (int i=0; i<ref_n; i++) {
    real_ref_cloud->points[i].getVector3fMap() =
      T_init.topLeftCorner(3,3) * real_ref_cloud->points[i].getVector3fMap() +
      T_init.topRightCorner(3,1);
  }
  */

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *real_src_cloud) == -1) {
    std::cout << "Error reading file " << argv[2] << std::endl;
    exit(1);
  }
  std::cout << "Read " << real_src_cloud->height << " clouds of " << real_src_cloud->width <<
    " points." << std::endl;
  int src_n = real_src_cloud->width;

  Eigen::Matrix<float, 4, 4> Trs = Eigen::Matrix<float, 4, 4>::Identity();

  std::vector<bool> matched;
  ICP(real_ref_cloud->makeShared(), real_src_cloud->makeShared(), Trs, 0.001,
      0.0001, 0.001, 50, &matched);
  std::cout << Trs << std::endl;

  pcl::PointCloud<pcl::PointXYZ> transformed_src;
  transformed_src.height = 1;
  transformed_src.width = src_n;
  transformed_src.is_dense = false;
  transformed_src.resize(src_n);

  for (int i=0; i<src_n; i++) {
    transformed_src.points[i].getVector3fMap() =
      Trs.topLeftCorner(3,3) * real_src_cloud->points[i].getVector3fMap() +
      Trs.topRightCorner(3,1);
  }

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
  SceneGraph::GLCachedPrimitives glPCsrr(GL_POINTS, SceneGraph::GLColor(0.4f, 0.4f, 0.4f));
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
      for (int i = 0; i < ref_n; i++) {
        glPCref.AddVertex(Eigen::Vector3d(
              real_ref_cloud->points[i].x,
              real_ref_cloud->points[i].y,
              real_ref_cloud->points[i].z));
      }
      for (int i = 0; i < src_n; i++) {
        glPCsrr.AddVertex(Eigen::Vector3d(
              real_src_cloud->points[i].x,
              real_src_cloud->points[i].y,
              real_src_cloud->points[i].z));
      }
      for (int i = 0; i < src_n; i++){
        if (matched[i]) {
          glPCsrc.AddVertex(Eigen::Vector3d(
                transformed_src.points[i].x,
                transformed_src.points[i].y,
                transformed_src.points[i].z));
        } else {
          glPCsru.AddVertex(Eigen::Vector3d(
                transformed_src.points[i].x,
                transformed_src.points[i].y,
                transformed_src.points[i].z));
        }
      }
      draw = false;
    }

    pangolin::FinishFrame();
  }
}
