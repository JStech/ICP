#include "kdtree.h"
#include <cstdlib>
#include <iostream> // TODO: remove this when done debugging
#include <iomanip>  // ditto

void KDTree::build_tree(point_vector &points, size_t start, size_t end,
    unsigned depth) {
  for (unsigned i=0; i<depth; i++) {
    std::cerr << "  ";
  }
  std::cerr << "build tree " << start << " " << end << " ";
  size_t left_size = (end - start)/2;
  size_t right_size = (end - start) - left_size - 1;
  this->depth = depth;
  this->bound = KDTree::select(points, depth%3, start, end, start+left_size);
  std::cerr << this->bound << std::endl;
  if (left_size > 0) {
    this->left = new KDTree();
    this->left->build_tree(points, start, start+left_size, depth+1);
  }
  if (right_size > 0) {
    this->right = new KDTree();
    this->left->build_tree(points, start+left_size+1, end, depth+1);
  }
}

void KDTree::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  this->build_tree(cloud->points, 0, cloud->points.size(), 0);
}

// select point at absolute position k, between start (included) and end
// (excluded)
float KDTree::select(point_vector points, int dim, size_t start, size_t end,
    size_t k) {
  if (start+1==end) {
    if (start==k) return points[start].data[dim];
    else {
      exit(1);
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
