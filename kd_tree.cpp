/*
 *  k-d tree implementation on pcl::PointCloud type
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdlib.h>

class KDtree {
  uint32_t depth;
  KDtree *left;
  KDtree *right;
  float location;

  public:
  KDtree(pcl::PointCloud<pcl::PointXYZ>ConstPtr cloud,
      std::vector<uint32_t> which, uint32_t depth)
    : depth(depth), left(NULL), right(NULL) {

      // base case
      if (which.length == 1) {
        location = cloud->points[which[0]].data[depth%3];
        return;
      }

      // extract given dimension
      std::vector<float> values;
      for (int i=0; i<which.length; i++) {
        values.push_back(cloud->points[which[i]].data[depth%3]);
      }

      // median algorithm
      int start, end, target, pivot;
      start = 0;
      end = values.length;
      target = end / 2;

      while (target != start && target != end-1) {
        pivot = (rand()%(end-start)) + start;
        float t = values[start];
        values[start] = values[pivot];
        values[pivot] = t;
        int i, j;
        i = start+1;
        j = end-1;
        while (i<j) {
          if (values[i] > values[pivot]) {
            t = values[i];
            values[i] = values[j];
            values[j] = t;
            j--;
          } else {
            i++;
          }
        }

      }

      std::vector<uint32_t> left_which;
      std::vector<uint32_t> right_which;
    }
}
