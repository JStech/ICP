#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <Eigen/Eigenvalues>
#include "icp.h"
#include <iostream> // TODO: remove this when done developing
#define MAX_ITER 20

using namespace pcl;

float choose_xi(std::vector<std::vector<float>> nearest_d,
    std::vector<bool> matched) {
  // TODO: implement this
  return 100.0;
}

Eigen::Matrix<float, 4, 4> localize(PointCloud<PointXYZ> reference,
    PointCloud<PointXYZ> source, std::vector<bool> matched) {
  // Compute matrices C1, C2, C3
  Eigen::Matrix<float, 4, 4> C1 = Eigen::Matrix<float, 4, 4>::Zero();
  Eigen::Matrix<float, 4, 4> C2 = Eigen::Matrix<float, 4, 4>::Zero();

  float W = 0.;
  for (int i=0; i<matched.size(); i++) {
    if (!matched[i]) continue;
    C1 += Quat<float>(reference.points[i]).Q().transpose() *
      Quat<float>(source.points[i]).W();
    C2 += Quat<float>(source.points[i]).W() -
      Quat<float>(reference.points[i]).Q();
    W += 1.;
  }

  C1 *= -2;
  C2 *= -2;

  Eigen::Matrix<float, 4, 4> A = .5 * (0.5/W * C2.transpose() * C2 - C1 - C1.transpose());

  Eigen::EigenSolver<Eigen::Matrix<float, 4, 4>> solver;
  solver.compute(A, true);
  std::cout << solver.eigenvalues().transpose() << std::endl;

  return A;
}

// input: reference and source point clouds, prior SE3 transform guess
// output: SE3 transform from source to reference, total error (of some sort TODO)
float ICP(PointCloud<PointXYZ>::Ptr reference, PointCloud<PointXYZ> source,
    Sophus::SE3d &Trs) {

  // initialize ICP parameters
  float D = 10.0;
  float Dmax = 20*D;

  // build k-d tree
  KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud(reference);

  // vectors to receive nearest neighbors
  std::vector<std::vector<int>> nearest_i(source.size(), std::vector<int>(1));
  std::vector<std::vector<float>> nearest_d(source.size(),
      std::vector<float>(1));

  std::vector<bool> matched(source.size());

  for (int iter=0; iter < MAX_ITER; iter++) {

    // find closest points
    for (int i=0; i<source.size(); i++) {
      if (kdtree.nearestKSearch(source.points[i], 1, nearest_i[i],
            nearest_d[i]) == 0) {
        nearest_i[i][0] = -1;
        nearest_d[i][0] = 0.;
      }
    }

    float mu=0.;
    float sigma=0.;
    int n=0;

    // choose which matches to use
    for (int i=0; i<source.size(); i++) {
      matched[i] = nearest_d[i][0] < Dmax;
      if (matched[i]) {
        mu += nearest_d[i][0];
        sigma += nearest_d[i][0]*nearest_d[i][0];
      }
      n++;
    }

    mu = mu/n;
    sigma = sqrt(sigma/n - mu*mu);

    if (mu < D) {
      Dmax = mu + 3*sigma;
    } else if (mu < 3*D) {
      Dmax = mu + 2*sigma;
    } else if (mu < 6*D) {
      Dmax = mu + sigma;
    } else {
      Dmax = choose_xi(nearest_d, matched);
    }

    for (int i=0; i<source.size(); i++) {
      matched[i] = nearest_d[i][0] < Dmax;
    }

    // compute motion


    // apply to all source points
  }
}
