#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include <Eigen/Eigenvalues>
#include "icp.h"
#include <algorithm>
#include <stdlib.h>
#include <omp.h>
#ifdef PROFILE
#include <chrono>
#include <iostream>
#endif
#define MAX_ITER 40

using namespace pcl;

float choose_xi(std::vector<std::vector<float>> nearest_d,
    std::vector<int> match_i) {
  unsigned num_bins = 25;

  // get estimate of max distance
  float max = 0.f;
  for (int n = 0; n<100; n++) {
    int r = rand()%match_i.size();
    if (nearest_d[r][0] > max) {
      max = nearest_d[r][0];
    }
  }
  max *= 1.05;

  // build histogram
  std::vector<unsigned int> counts(num_bins+1, 0);
  for (size_t i=0; i<match_i.size(); i++) {
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

Eigen::Matrix4f localize(PointCloud<PointXYZ>::Ptr reference,
    PointCloud<PointXYZ>::Ptr source, std::vector<int> match_i) {

  // find centroids
  Eigen::Vector3f src_centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f ref_centroid = Eigen::Vector3f::Zero();
  int c = 0;
  for (size_t i=0; i<match_i.size(); i++) {
    if (match_i[i]==-1) continue;
    src_centroid += source->points[i].getVector3fMap();
    ref_centroid += reference->points[match_i[i]].getVector3fMap();
    c++;
  }
  src_centroid /= c;
  ref_centroid /= c;

  // estimate scale transform
  // scale variable is source/reference
  double scale = 0;
  for (size_t i=0; i<match_i.size(); i++) {
    if (match_i[i]==-1) continue;
    scale += (source->points[i].getVector3fMap() - src_centroid).norm() /
      (reference->points[match_i[i]].getVector3fMap() - ref_centroid).norm();
  }
  scale /= c;

  // perform SVD to recover rotation matrix
  // calculate cross correlation
  Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
  for (size_t i=0; i<match_i.size(); i++) {
    if (match_i[i]==-1) continue;
    pcl::PointXYZ &s_pt = source->points[i];
    pcl::PointXYZ &r_pt = reference->points[match_i[i]];
    M += (s_pt.getVector3fMap() - src_centroid)/scale * (r_pt.getVector3fMap()
        - ref_centroid).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(M, Eigen::ComputeFullU|Eigen::ComputeFullV);
  Eigen::Matrix4f R = Eigen::Matrix4f::Identity();
  R.topLeftCorner(3, 3) = (svd.matrixU() * (svd.matrixV().transpose())).transpose()/scale;

  // put it all together
  Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
  T1.topRightCorner(3, 1) = -src_centroid;
  Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
  T2.topRightCorner(3, 1) = ref_centroid;
  return T2*R*T1;
}

// input: reference and source point clouds, prior SE3 transform guess
// output: SE3 transform from source to reference, total error (of some sort TODO)
float ICP(PointCloud<PointXYZ>::Ptr reference, PointCloud<PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs, float D, std::vector<bool> *matched) {

  // initialize ICP parameters
  float Dmax = 20*D;

  // transformations calculated at each iteration
  Eigen::Matrix<float, 4, 4> Tmat;

#ifdef PROFILE
  std::chrono::duration<uint64_t, std::micro> kdtree_build_time(0);
  std::chrono::duration<uint64_t, std::micro> kdtree_search_time(0);
  std::chrono::duration<uint64_t, std::micro> match_time(0);
  std::chrono::duration<uint64_t, std::micro> localize_time(0);
  std::chrono::duration<uint64_t, std::micro> update_time(0);
  auto start = std::chrono::high_resolution_clock::now();
#endif
  // build k-d tree
  KdTreeFLANN<PointXYZ> kdtree;
  kdtree.setInputCloud(reference);
#ifdef PROFILE
  auto stop = std::chrono::high_resolution_clock::now();
  kdtree_build_time = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  start = stop;
#endif

  // vectors to receive nearest neighbors
  std::vector<std::vector<int>> nearest_i(source->size(), std::vector<int>(1));
  std::vector<std::vector<float>> nearest_d(source->size(),
      std::vector<float>(1));

  std::vector<int> match_i(source->size());

  // transform source points according to prior Trs
  for (size_t i=0; i<source->size(); i++) {
    source->points[i].getVector4fMap() = Trs*source->points[i].getVector4fMap();
  }

  for (int iter=0; iter < MAX_ITER; iter++) {

    // find closest points
#pragma omp parallel for
    for (size_t i=0; i<source->size(); i++) {
      nearest_i[i][0] = -1;
      nearest_d[i][0] = INFINITY;
      if (!isnan(source->points[i].x)) {
        kdtree.nearestKSearch(source->points[i], 1, nearest_i[i],
            nearest_d[i]);
      }
    }
#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    kdtree_search_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif

    // choose which matches to use
    float mu=0.;
    float sigma=0.;
    int n=0;

    for (size_t i=0; i<source->size(); i++) {
      if (nearest_d[i][0] < Dmax) {
        match_i[i] = nearest_i[i][0];
        mu += nearest_d[i][0];
        sigma += nearest_d[i][0]*nearest_d[i][0];
        n++;
      } else {
        match_i[i] = -1;
      }
    }

    mu = mu/n;
    sigma = sqrt(sigma/n - mu*mu);

    std::cerr << "mu " << mu << ", D " << D << std::endl;
    if (mu < D) {
      Dmax = mu + 3*sigma;
    } else if (mu < 3*D) {
      Dmax = mu + 2*sigma;
    } else if (mu < 6*D) {
      Dmax = mu + sigma;
    } else {
      Dmax = choose_xi(nearest_d, match_i);
    }

#ifdef PROFILE
    int matched_n = 0;
    int tot_n = 0;
#endif
    for (size_t i=0; i<source->size(); i++) {
      if (nearest_d[i][0] < Dmax) {
        match_i[i] = nearest_i[i][0];
#ifdef PROFILE
        matched_n++;
        tot_n++;
#endif
      } else {
        match_i[i] = -1;
#ifdef PROFILE
        if (!isnan(source->points[i].x)) tot_n++;
#endif
      }
    }
#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    match_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif

    // compute motion
    Tmat = localize(reference, source, match_i);

#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    localize_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif
    // apply to all source points
    for (size_t i=0; i<source->points.size(); i++) {
      source->points[i].getVector4fMap() = Tmat*source->points[i].getVector4fMap();
    }

    // update Trs
    Trs = Tmat*Trs;

    // check stopping criteria
    float scale = Tmat.block(0, 0, 3, 1).norm();
    float dt = Tmat.topRightCorner(3, 1).norm();
    float dth = acos((Tmat.topLeftCorner(3, 3).trace()/scale - 1.)/2.);

#ifdef PROFILE
    std::cerr << "Iteration " << iter << " dt " << dt << ", dtheta " << dth <<
      ", scale " << scale << ", matched " << matched_n << "/" << tot_n << std::endl;
    stop = std::chrono::high_resolution_clock::now();
    update_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif
    if (iter > 0 && dt < 0.01 && dth < 0.01) {
      break;
    }
  }

  if (matched != NULL) {
    matched->resize(match_i.size());
    for (size_t i = 0; i<match_i.size(); i++) {
      (*matched)[i] = match_i[i]>0;
    }
  }

#ifdef PROFILE
  std::cerr << "kdtree_build_time  " << kdtree_build_time.count() << std::endl;
  std::cerr << "kdtree_search_time " << kdtree_search_time.count() << std::endl;
  std::cerr << "match_time         " << match_time.count() << std::endl;
  std::cerr << "localize_time      " << localize_time.count() << std::endl;
  std::cerr << "update_time        " << update_time.count() << std::endl;
#endif

  return 0.f;
}
