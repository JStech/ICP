#ifndef ICP_H
#define ICP_H

float ICP(PointCloud<PointXYZ> reference, PointCloud<PointXYZ> source,
    Sophus::SE3d &Trs);

#endif
