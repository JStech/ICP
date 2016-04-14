#include "kdtree.h"
#include <cstdlib>
#include <math.h>

/////// recursive tree-building function
// considers the subarray of points between start (included) and end (excluded)
// partitions into two equal subarrays according to the dimension determined by
// depth%3 (if subarray is of odd length, the extra element goes before the
// partition); then recurses on subarrays;
// sets this->bound to be the largest value of the smaller subarray
void KDTree::build_tree(point_vector points,
    std::vector<std::vector<int>>& indices, std::vector<bool> active,
    size_t n, unsigned depth) {
  this->depth = depth;

  int median = n/2;
  std::vector<bool> left_active(active.size(), false);
  size_t left_n = 0;
  std::vector<bool> right_active(active.size(), false);
  size_t right_n = 0;

  int c=0;
  for (size_t i=0; i<points.size(); i++) {
    if (!active[indices[depth%3][i]]) continue;
    if (c<median) {
      left_active[indices[depth%3][i]] = true;
      left_n++;
    } else if (c == median) {
      this->point = points[indices[depth%3][i]];
      this->point_i = indices[depth%3][i];
      this->bound = points[indices[depth%3][i]].data[depth%3];
      active[indices[depth%3][i]] = false;
    } else {
      right_active[indices[depth%3][i]] = true;
      right_n++;
    }
    c++;
  }

  if (left_n > 0) {
    this->l = new KDTree();
    this->l->build_tree(points, indices, left_active, left_n, depth+1);
  }
  if (right_n > 0) {
    this->r = new KDTree();
    this->r->build_tree(points, indices, right_active, right_n, depth+1);
  }
}

// sort points along dimension dim
void KDTree::presort(point_vector points, std::vector<int>& indices,
    size_t start, size_t end, int dim) {
  if (start+1 >= end) {
    return;
  }

  size_t pivot = start + rand()%(end - start);
  int t = indices[pivot];
  indices[pivot] = indices[start];
  indices[start] = t;

  size_t i = start+1;
  size_t j = end-1;
  while (i<j) {
    if (points[indices[i]].data[dim] < points[indices[start]].data[dim]) {
      i++;
    } else {
      t = indices[i];
      indices[i] = indices[j];
      indices[j] = t;
      j--;
    }
  }

  if (points[indices[i]].data[dim] > points[indices[start]].data[dim]) {
    i--;
  }
  t = indices[i];
  indices[i] = indices[start];
  indices[start] = t;

  KDTree::presort(points, indices, start, i, dim);
  KDTree::presort(points, indices, i+1, end, dim);
}

void KDTree::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::vector<std::vector<int>> indices(3);
  int n = cloud->points.size();
  for (int dim=0; dim<3; dim++) {
    indices[dim].resize(n);
    for (int i=0; i<n; i++) {
      indices[dim][i] = i;
    }
    this->presort(cloud->points, indices[dim], 0, n, dim);
  }
  std::vector<bool> active(n, true);
  this->build_tree(cloud->points, indices, active, n, 0);
}

// http://andrewd.ces.clemson.edu/courses/cpsc805/references/nearest_search.pdf
void KDTree::search(pcl::PointXYZ target_point, KDTree* node,
      int& nearest_i, float& nearest_d2) {
  if (node == NULL) return;

  int dim = node->depth%3;
  float xyz = target_point.data[dim];

  if (xyz < node->bound) {
    // search left subtree first
    search(target_point, node->l, nearest_i, nearest_d2);
    if ((xyz - node->bound)*(xyz - node->bound) < nearest_d2) {
      float d2 = dist2(target_point, node->point);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        nearest_i = node->point_i;
      }
      search(target_point, node->r, nearest_i, nearest_d2);
    }
  } else {
    // search right subtree first
    search(target_point, node->r, nearest_i, nearest_d2);
    if ((xyz - node->bound)*(xyz - node->bound) < nearest_d2) {
      float d2 = dist2(target_point, node->point);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        nearest_i = node->point_i;
      }
      search(target_point, node->l, nearest_i, nearest_d2);
    }
  }
}

void KDTree::nearestKSearch(pcl::PointXYZ point, int k, std::vector<int>& nearest_i,
        std::vector<float>& nearest_d) {
  // just implementing 1-nearest neighbor, but keeping the argument to match
  // PCL's KDTree interface
  if (k!=1) return;

  nearest_i.resize(1);
  nearest_d.resize(1);
  nearest_d[0] = INFINITY;
  this->search(point, this, nearest_i[0], nearest_d[0]);
  nearest_d[0] = sqrt(nearest_d[0]);
}
