#ifndef POINT_XYZIRZ_H_
#define POINT_XYZIRZ_H_

#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

struct PointXYZRID // This struct gets used as an array later, so DO NOT USE ANY OTHER TYPE THAN FLOAT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float ring;
  float intensity;
  float range;
  float noise;
  float reflectivity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRID,           // here we assume a XYZ + "scan ring" + "scan intensity" + "scan distance" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, ring, ring)
                                   (float, intensity, intensity)
                                   (float, range, range)
                                   (float, noise, noise)
                                   (float, reflectivity, reflectivity)
)

#endif
