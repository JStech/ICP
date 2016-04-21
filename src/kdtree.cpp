#include "kdtree.h"
#include <cstdlib>
#include <math.h>

/////// recursive tree-building function
// considers the subarray of points between start (included) and end (excluded)
// partitions into two equal subarrays according to the dimension determined by
// depth%3 (if subarray is of odd length, the extra element goes before the
// partition); then recurses on subarrays;
// sets this->bound to be the largest value of the smaller subarray
void KDTree::build_tree(point_vector& points, std::vector<int>& indices,
    size_t start, size_t end, unsigned depth) {
  size_t median = (end - start)/2 + start;
  this->depth = depth;
  this->bound = KDTree::select(points, indices, depth%3, start, end, median);
  this->point = points[indices[median]];
  this->point_i = indices[median];
  if (median-start > 0) {
    this->l = new KDTree();
    this->l->build_tree(points, indices, start, median, depth+1);
  }
  if (end-(median+1) > 0) {
    this->r = new KDTree();
    this->r->build_tree(points, indices, median+1, end, depth+1);
  }
}

void KDTree::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::vector<int> indices;
  indices.resize(cloud->points.size());
  for (int i=0; i<(int) cloud->points.size(); i++) {
    indices[i] = i;
  }
  this->build_tree(cloud->points, indices, 0, cloud->points.size(), 0);
}

// select point at (sorted) absolute position k (and place it there),
// considering only points between start (included) and end (excluded); it is
// assumed that points less than start/greater than or equal to end are known
// to be less than/greater than position k
float KDTree::select(point_vector& points, std::vector<int>& indices, int dim,
    size_t start, size_t end, size_t k) {
  if (start+1==end) {
    if (start==k) return points[indices[start]].data[dim];
    else {
      // this shouldn't happen
      exit(255);
    }
  }
  size_t pivot = start + rand()%(end - start);
  int t = indices[pivot];
  indices[pivot] = indices[start];
  indices[start] = t;

  size_t i = start+1;
  size_t j = end-1;
  while (i < j) {
    if (points[indices[i]].data[dim] < points[indices[start]].data[dim]) {
      i++;
    } else {
      t = indices[i];
      indices[i] = indices[j];
      indices[j] = t;
      j--;
    }
  }

  // make i point to last element smaller than pivot
  if (points[indices[i]].data[dim] > points[indices[start]].data[dim]) {
    i--;
  }
  t = indices[i];
  indices[i] = indices[start];
  indices[start] = t;

  if (i==k) {
    return points[indices[i]].data[dim];
  } else if (i<k) {
    return KDTree::select(points, indices, dim, i+1, end, k);
  } else {
    return KDTree::select(points, indices, dim, start, i, k);
  }
}

void KDTree::search(pcl::PointXYZ target_point, KDTree* node,
      int& nearest_i, float& nearest_d2, float bounds[6], bool& done) {
  if (node == NULL) return;

  int dim = node->depth%3;
  float xyz = target_point.data[dim];
  float t;

  if (xyz < node->bound) {
    // search left subtree first
    t = bounds[dim*2 + 1];
    bounds[dim*2 + 1] = xyz;
    search(target_point, node->l, nearest_i, nearest_d2, bounds, done);
    if (done) return;

    // check if we need to search right subtree
    if ((xyz - node->bound)*(xyz - node->bound) < nearest_d2) {
      float d2 = dist2(target_point, node->point);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        nearest_i = node->point_i;
      }
      bounds[dim*2+1] = t;
      bounds[dim*2+0] = xyz;
      search(target_point, node->r, nearest_i, nearest_d2, bounds, done);
      if (done) return;
    }
  } else {
    // search right subtree first
    t = bounds[dim*2 + 0];
    bounds[dim*2 + 0] = xyz;
    search(target_point, node->r, nearest_i, nearest_d2, bounds, done);
    if (done) return;

    // check if we need to search left subtree
    if ((xyz - node->bound)*(xyz - node->bound) < nearest_d2) {
      float d2 = dist2(target_point, node->point);
      if (d2 < nearest_d2) {
        nearest_d2 = d2;
        nearest_i = node->point_i;
      }
      bounds[dim*2+0] = t;
      bounds[dim*2+1] = xyz;
      search(target_point, node->l, nearest_i, nearest_d2, bounds, done);
      if (done) return;
    }
  }

  // check if ball around target point lies entirely within current bounds
  done = (
      ((bounds[0] - target_point.x)*(bounds[0] - target_point.x) > nearest_d2) &&
      ((bounds[1] - target_point.x)*(bounds[1] - target_point.x) > nearest_d2) &&
      ((bounds[2] - target_point.y)*(bounds[2] - target_point.y) > nearest_d2) &&
      ((bounds[3] - target_point.y)*(bounds[3] - target_point.y) > nearest_d2) &&
      ((bounds[4] - target_point.z)*(bounds[4] - target_point.z) > nearest_d2) &&
      ((bounds[5] - target_point.z)*(bounds[5] - target_point.z) > nearest_d2));
}

void KDTree::nearestKSearch(pcl::PointXYZ point, int k, std::vector<int>& nearest_i,
        std::vector<float>& nearest_d) {
  // just implementing 1-nearest neighbor, but keeping the argument to match
  // PCL's KDTree interface
  if (k!=1) return;

  nearest_i.resize(1);
  nearest_d.resize(1);
  nearest_d[0] = INFINITY;
  float bounds[6] = {
    -INFINITY, INFINITY,
    -INFINITY, INFINITY,
    -INFINITY, INFINITY};
  bool done = false;
  this->search(point, this, nearest_i[0], nearest_d[0], bounds, done);
  nearest_d[0] = sqrt(nearest_d[0]);
}
