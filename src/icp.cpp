//#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <Eigen/Eigenvalues>
#include "icp.h"
#include "dualquat.h"
#include <algorithm>
#include <stdlib.h>
#define MAX_ITER 40

using namespace pcl;

float choose_xi(std::vector<std::vector<float>> nearest_d,
    std::vector<int> matched) {
  unsigned num_bins = 25;

  // get estimate of max distance
  float max = 0.f;
  for (int n = 0; n<100; n++) {
    int r = rand()%matched.size();
    if (nearest_d[r][0] > max) {
      max = nearest_d[r][0];
    }
  }
  max *= 1.05;

  // build histogram
  std::vector<unsigned int> counts(num_bins+1, 0);
  for (size_t i=0; i<matched.size(); i++) {
    if (nearest_d[i][0] > max) {
      counts[num_bins]++;
    } else {
      counts[(int)(num_bins*nearest_d[i][0]/max)]++;
    }
  }

  // find biggest peak
  unsigned peak = 0;
  unsigned elevation = 0;
  for (size_t i=0; i<num_bins+1; i++) {
    if (counts[i] > elevation) {
      peak = i;
      elevation = counts[i];
    }
  }

  // find first valley after peak (lower than 60% of peak height)
  unsigned valley;
  for (valley=peak+1; valley<num_bins; valley++) {
    if (counts[valley] > elevation*0.6) continue;
    if (counts[valley+1] > counts[valley]) break;
  }

  return ((float) valley)/((float) num_bins) * max;
}

Eigen::Matrix<float, 4, 4> localize(PointCloud<PointXYZ>::Ptr reference,
    PointCloud<PointXYZ>::Ptr source, std::vector<int> matched) {
  // Compute matrices C1, C2, C3
  Eigen::Matrix<float, 4, 4> C1 = Eigen::Matrix<float, 4, 4>::Zero();
  Eigen::Matrix<float, 4, 4> C2 = Eigen::Matrix<float, 4, 4>::Zero();

  float W = 0.;
  for (size_t i=0; i<matched.size(); i++) {
    if (matched[i]<0) continue;
    C1 += Quat<float>(reference->points[matched[i]]).Q().transpose() *
      Quat<float>(source->points[i]).W();
    C2 += Quat<float>(source->points[i]).W() -
      Quat<float>(reference->points[matched[i]]).Q();
    W += 1.;
  }

  C1 *= -2;
  C2 *= -2;

  Eigen::Matrix<float, 4, 4> A = .5 * (0.5/W * C2.transpose() * C2 - C1 - C1.transpose());

  Eigen::EigenSolver<Eigen::Matrix<float, 4, 4>> solver;
  solver.compute(A, true);
  float max_eigenvalue = -INFINITY;
  Eigen::Matrix<float, 4, 1> max_eigenvector;
  for (long i = 0; i < solver.eigenvalues().size(); i++) {
    if (solver.eigenvalues()[i].real() > max_eigenvalue) {
      max_eigenvalue = solver.eigenvalues()[i].real();
      max_eigenvector = solver.eigenvectors().col(i).real();
    }
  }

  Eigen::Matrix<float, 4, 1> s = 1./W * C2 * max_eigenvector;
  Eigen::Matrix<float, 4, 1> t = Quat<float>(max_eigenvector).W().transpose() * s;

  A.block(0, 0, 3, 3) = Quat<float>(max_eigenvector).Rot();
  A.block(0, 3, 3, 1) = t.block(0, 0, 3, 1);
  A.block(3, 0, 1, 4) << 0., 0., 0., 1.;
  return A;
}

// input: reference and source point clouds, prior SE3 transform guess
// output: SE3 transform from source to reference, total error (of some sort TODO)
float ICP(PointCloud<PointXYZ>::Ptr reference, PointCloud<PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs) {

  // initialize ICP parameters
  float D = 10.0;
  float Dmax = 20*D;

  // build k-d tree
  KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud(reference);

  // vectors to receive nearest neighbors
  std::vector<std::vector<int>> nearest_i(source->size(), std::vector<int>(1));
  std::vector<std::vector<float>> nearest_d(source->size(),
      std::vector<float>(1));

  std::vector<int> matched(source->size());

  // transform source points according to prior Trs
  for (size_t i=0; i<source->size(); i++) {
    source->points[i].getVector4fMap() = Trs*source->points[i].getVector4fMap();
  }

  for (int iter=0; iter < MAX_ITER; iter++) {

    // find closest points
    for (size_t i=0; i<source->size(); i++) {
      nearest_i[i][0] = -1;
      nearest_d[i][0] = 0.;
      if (!isnan(source->points[i].x)) {
        kdtree.nearestKSearch(source->points[i], 1, nearest_i[i],
            nearest_d[i]);
      }
    }

    // choose which matches to use
    float mu=0.;
    float sigma=0.;
    int n=0;

    for (size_t i=0; i<source->size(); i++) {
      if (nearest_d[i][0] < Dmax) {
        matched[i] = nearest_i[i][0];
        mu += nearest_d[i][0];
        sigma += nearest_d[i][0]*nearest_d[i][0];
      } else {
        matched[i] = -1;
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

    for (size_t i=0; i<source->size(); i++) {
      if (nearest_d[i][0] < Dmax) {
        matched[i] = nearest_i[i][0];
      } else {
        matched[i] = -1;
      }
    }

    // compute motion
    Eigen::Matrix<float, 4, 4> T = localize(reference, source, matched);

    // apply to all source points
    for (size_t i=0; i<source->points.size(); i++) {
      source->points[i].getVector4fMap() = T*source->points[i].getVector4fMap();
    }

    // update Trs
    Trs = T*Trs;
  }

  return 0.f;
}
