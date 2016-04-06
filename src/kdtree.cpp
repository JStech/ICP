#include "kdtree.h"
#include <cstdlib>
#include <iostream> // TODO: remove this when done debugging
#include <iomanip>  // ditto
#include <stdio.h>  // ditto

/////// recursive tree-building function
// considers the subarray of points between start (included) and end (excluded)
// partitions into two equal subarrays according to the dimension determined by
// depth%3 (if subarray is of odd length, the extra element goes before the
// partition); then recurses on subarrays;
// sets this->bound to be the largest value of the smaller subarray
void KDTree::build_tree(point_vector &points, size_t start, size_t end,
    unsigned depth) {
  size_t median = (end - start)/2 + start;
  this->depth = depth;
  this->bound = KDTree::select(points, depth%3, start, end, median);
  this->point = points[median];
  if (median-start > 0) {
    this->l = new KDTree();
    this->l->build_tree(points, start, median, depth+1);
  }
  if (end-(median+1) > 0) {
    this->r = new KDTree();
    this->r->build_tree(points, median+1, end, depth+1);
  }
}

void KDTree::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  this->build_tree(cloud->points, 0, cloud->points.size(), 0);
}

// select point at (sorted) absolute position k (and place it there),
// considering only points between start (included) and end (excluded); it is
// assumed that points less than start/greater than or equal to end are known
// to be less than/greater than position k
float KDTree::select(point_vector& points, int dim, size_t start, size_t end,
    size_t k) {
  if (start+1==end) {
    if (start==k) return points[start].data[dim];
    else {
      // this shouldn't happen
      exit(255);
    }
  }
  size_t pivot = start + rand()%(end - start);
  swap(points[pivot], points[start]);

  size_t i = start+1;
  size_t j = end-1;
  while (i < j) {
    if (points[i].data[dim] < points[start].data[dim]) {
      i++;
    } else {
      swap(points[i], points[j]);
      j--;
    }
  }

  // make i point to last element smaller than pivot
  if (points[i].data[dim] > points[start].data[dim]) {
    i--;
  }
  swap(points[i], points[start]);

  if (i==k) {
    return points[i].data[dim];
  } else if (i<k) {
    return KDTree::select(points, dim, i+1, end, k);
  } else {
    return KDTree::select(points, dim, start, i, k);
  }
}

void KDTree::nearestKSearch(pcl::PointXYZ point, int k, std::vector<int> nearest_i,
        std::vector<float> nearest_d) {
  // TODO
}
