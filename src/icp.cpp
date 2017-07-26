#include <pcl/compression/octree_pointcloud_compression.h>
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
#include <cassert>
#endif
#define MAX_ITER 40

using namespace pcl;

template<typename T>
std::vector<T> flatten(const std::vector<std::vector<T>> &orig) {
  std::vector<T> ret;
  for (const auto &e: orig) {
    ret.insert(ret.end(), e.begin(), e.end());
  }
  return ret;
}

// return value at position k in list L
float quickselect(std::vector<float> L, size_t k) {
  size_t n = L.size();
  assert(0<=k);
  assert(k<n);
  std::vector<float> lt_items;
  std::vector<float> gt_items;
  float pivot = L[rand()%n];
  for (size_t i=0; i<n; i++) {
    if (L[i] < pivot) {
      lt_items.push_back(L[i]);
    } else if (L[i] > pivot) {
      gt_items.push_back(L[i]);
    }
  }
  if (k < lt_items.size()) {
    return quickselect(lt_items, k);
  } else if (k >= n-gt_items.size()) {
    return quickselect(gt_items, k - (n-gt_items.size()));
  } else {
    return pivot;
  }
}

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
    PointCloud<PointXYZ>::Ptr source, std::vector<int> match_i,
    bool do_scale) {

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

  double scale = 0;
  if (do_scale) {
    // estimate scale transform
    // scale variable is source/reference
    for (size_t i=0; i<match_i.size(); i++) {
      if (match_i[i]==-1) continue;
      scale += (source->points[i].getVector3fMap() - src_centroid).norm() /
        (reference->points[match_i[i]].getVector3fMap() - ref_centroid).norm();
    }
    scale /= c;
  } else {
    scale = 1.0;
  }

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

std::vector<int> find_matches(float beta, std::vector<int> idx,
    std::vector<float> dist, std::vector<int> init) {
  const int h = 480;
  const int w = 640;
  assert(idx.size() == h*w);
  assert(dist.size() == h*w);
  assert(init.size() == h*w);
  const int max_iters = 3;
  double in_mean = 0.;
  double in_std = 0.;
  int in_n = 0;
  double out_mean = 0.;
  double out_std = 0.;
  int out_n = 0;

  for (size_t i=0; i<idx.size(); i++) {
    if (init[i]>=0) {
      in_mean += dist[i];
      in_std += dist[i]*dist[i];
      in_n++;
    } else {
      out_mean += dist[i];
      out_std += dist[i]*dist[i];
      out_n++;
    }
  }
  in_mean = in_mean/in_n;
  in_std = std::sqrt(in_std/in_n - in_mean*in_mean);
  out_mean = out_mean/out_n;
  out_std = std::sqrt(out_std/out_n - out_mean*out_mean);

  std::vector< std::vector<float> > z(h, std::vector<float>(w));

  if (std::isnan(in_mean) || std::isnan(in_std) || std::isnan(out_mean) ||
      std::isnan(out_std)) {
    std::cerr << "NaN! in_mean: " << in_mean << ", in_std: " << in_std <<
      ", out_mean: " << out_mean << ", out_std: " << out_std << std::endl;
    return std::vector<int>();
  }

  // E-step
  for (int iters=0; iters<max_iters; iters++) {
    for (int i=0; i<h; i++) {
      for (int j=0; j<w; j++) {
        int ij = i*w+j;
        float mean_field = 0.;
        if (i>0) mean_field += z[i-1][j];
        if (i<h-1) mean_field += z[i+1][j];
        if (j>0) mean_field += z[i][j-1];
        if (j<w-1) mean_field += z[i][j+1];
        mean_field/=4;
        float r_in = beta*mean_field - std::log(in_std) -
          (dist[ij] - in_mean)*(dist[ij] - in_mean)/(2*in_std*in_std);
        float r_out = beta*mean_field - std::log(out_std) -
          (dist[ij] - out_mean)*(dist[ij] - out_mean)/(2*out_std*out_std);
        z[i][j] = 2*std::exp(r_in) / (std::exp(r_out) + std::exp(r_in)) - 1;
      }
    }

    // M-step
    in_mean = 0;
    in_std = 0;
    out_mean = 0;
    out_std = 0;
    float in_sum = 0.;
    float out_sum = 0.;
    for (int i=0; i<h; i++) {
      for (int j=0; j<w; j++) {
        int ij = i*w+j;
        in_mean += dist[ij] * (1+z[i][j])/2;
        in_std += dist[ij]*dist[ij] * (1+z[i][j])/2;
        in_sum += (1+z[i][j])/2;
        out_mean += dist[ij] * (1-z[i][j])/2;
        out_std += dist[ij]*dist[ij] * (1-z[i][j])/2;
        out_sum += (1-z[i][j])/2;
      }
    }
    in_mean = in_mean/in_sum;
    in_std = std::sqrt(in_std/in_sum - in_mean*in_mean);
    out_mean = out_mean/out_sum;
    out_std = std::sqrt(out_std/out_sum - out_mean*out_mean);
  }

  std::vector<int> matches(h*w);
  for (int i=0; i<h; i++) {
    for (int j=0; j<w; j++) {
      int ij = i*w+j;
      if (z[i][j] < 0) {
        matches[ij] = -1;
      } else {
        matches[ij] = idx[ij];
      }
    }
  }

  return matches;
}

// input: reference and source point clouds, prior SE3 transform guess
// output: SE3 transform from source to reference, total error (of some sort TODO)
float ICP_hmrf(PointCloud<PointXYZ>::Ptr reference, PointCloud<PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs, float beta, float dt_thresh, float dth_thresh,
    int max_iter, std::vector<bool> *matched) {

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
  std::vector<std::vector<int>> nearest_i_t(source->size(), std::vector<int>(1));
  std::vector<std::vector<float>> nearest_d_t(source->size(),
      std::vector<float>(1));

  // transform source points according to prior Trs
  for (size_t i=0; i<source->size(); i++) {
    source->points[i].getVector3fMap() =
      Trs.topLeftCorner(3,3)*source->points[i].getVector3fMap() +
      Trs.topRightCorner(3,1);
  }

  std::vector<int> matches(source->size());
  for (int iter=0; iter < MAX_ITER; iter++) {

    // find closest points
#pragma omp parallel for
    for (size_t i=0; i<source->size(); i++) {
      nearest_i_t[i][0] = -1;
      nearest_d_t[i][0] = INFINITY;
      if (!isnan(source->points[i].x)) {
        kdtree.nearestKSearch(source->points[i], 1, nearest_i_t[i],
            nearest_d_t[i]);
      }
    }

    std::vector<int> nearest_i = flatten<int>(nearest_i_t);
    std::vector<float> nearest_d = flatten<float>(nearest_d_t);

    // initialize matches
    if (iter==0) {
      float threshold = quickselect(nearest_d, 0.9*nearest_d.size());
      for (size_t i=0; i<matches.size(); i++) {
        if (nearest_d[i] > threshold) {
          matches[i] = -1;
        } else {
          matches[i] = nearest_i[i];
        }
      }
    }

#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    kdtree_search_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif

    // choose which matches to use
    matches = find_matches(beta, nearest_i, nearest_d, matches);
#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    match_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif

    // compute motion
    Tmat = localize(reference, source, matches, true);

#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    localize_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif
    // apply to all source points
    for (size_t i=0; i<source->points.size(); i++) {
      source->points[i].getVector3fMap() =
        Tmat.topLeftCorner(3,3)*source->points[i].getVector3fMap() +
        Tmat.topRightCorner(3,1);
    }

    // update Trs
    Trs = Tmat*Trs;

    // check stopping criteria
    float scale = Tmat.block(0, 0, 3, 1).norm();
    float dt = Tmat.topRightCorner(3, 1).norm();
    float dth = acos((Tmat.topLeftCorner(3, 3).trace()/scale - 1.)/2.);

#ifdef PROFILE
    std::cerr << "Iteration " << iter << " dt " << dt << ", dtheta " << dth <<
      ", scale " << scale << std::endl;
    stop = std::chrono::high_resolution_clock::now();
    update_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif
    if (iter >= max_iter || (iter > 0 && dt < dt_thresh && dth < dth_thresh)) {
      break;
    }
  }

  if (matched != NULL) {
    matched->resize(matches.size());
    for (size_t i = 0; i<matches.size(); i++) {
      (*matched)[i] = matches[i]>=0;
    }
  }

#ifdef PROFILE
  std::cerr << "kdtree_build_time  " << kdtree_build_time.count() << std::endl;
  std::cerr << "kdtree_search_time " << kdtree_search_time.count() << std::endl;
  std::cerr << "match_time         " << match_time.count() << std::endl;
  std::cerr << "localize_time      " << localize_time.count() << std::endl;
  std::cerr << "update_time        " << update_time.count() << std::endl;
#endif

  return 0.;
}

// input: reference and source point clouds, prior SE3 transform guess
// output: SE3 transform from source to reference, total error (of some sort TODO)
float ICP_zhang(PointCloud<PointXYZ>::Ptr reference, PointCloud<PointXYZ>::Ptr source,
    Eigen::Matrix<float, 4, 4> &Trs, float D, float dt_thresh, float dth_thresh,
    int max_iter, std::vector<bool> *matched) {

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
    source->points[i].getVector3fMap() =
      Trs.topLeftCorner(3,3)*source->points[i].getVector3fMap() +
      Trs.topRightCorner(3,1);
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
    Tmat = localize(reference, source, match_i, true);

#ifdef PROFILE
    stop = std::chrono::high_resolution_clock::now();
    localize_time += std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    start = stop;
#endif
    // apply to all source points
    for (size_t i=0; i<source->points.size(); i++) {
      source->points[i].getVector3fMap() =
        Tmat.topLeftCorner(3,3)*source->points[i].getVector3fMap() +
        Tmat.topRightCorner(3,1);
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
    if (iter >= max_iter || (iter > 0 && dt < dt_thresh && dth < dth_thresh)) {
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

void downsample_cloud(float d,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud) {
  pcl::io::compression_Profiles_e compression_profile = pcl::io::MANUAL_CONFIGURATION;

  auto PointCloudEncoder = new
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> (compression_profile,
        false, d, d, true, 50, true, 4);
  auto PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ();

  std::stringstream compressedData;

  PointCloudEncoder->encodePointCloud(in_cloud, compressedData);
  PointCloudDecoder->decodePointCloud(compressedData, out_cloud);
}
