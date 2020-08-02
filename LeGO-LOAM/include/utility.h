#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "cloud_msgs/cloud_info.h"

#define PCL_NO_PRECOMPILE

#include <opencv/cv.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <slam_point.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

const double DEG_TO_RAD = PI / 180.0;

typedef PointXYZRID  PointType;

//extern const string pointCloudTopic = "/ALBERT/points";
//extern const string imuTopic = "/imu/data";
//extern const string imuTopic = "/ALBERT/imu/data";

// Save pcd
//extern const string fileDirectory = "/tmp/";

// VLP-16
/*extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;*/

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet, please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

/*extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 1.0/15.0;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;
extern const float imuGravity = 9.8;

extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;


// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized
*/

inline void pntFillExtra(PointType *input, PointType *output) {
  vector<pcl::PCLPointField> fields;
  pcl::getFields<PointType>(fields);
  
  int ringIdx = -1;
  for(int i = 0; i < fields.size() && ringIdx == -1; i++) {
    if(fields[i].name == "ring") ringIdx = i;
  }
  
  std::uint8_t* in_data = reinterpret_cast<std::uint8_t*>(input);
  std::uint8_t* out_data = reinterpret_cast<std::uint8_t*>(output);
  for(int i = ringIdx + 1; i < fields.size(); i++) {
    memcpy(out_data + fields[i].offset, in_data + fields[i].offset, sizeof(float));
  }
}

inline void downSizeCustomPoint(pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter, pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud) {
  if(inputCloud->points.size() == 0) {
    outputCloud->points.clear();
    return;
  }
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDS(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.resize(inputCloud->points.size());
  for(int i = 0; i < inputCloud->points.size(); i++) {
    cloud->points[i].x = inputCloud->points[i].x;
    cloud->points[i].y = inputCloud->points[i].y;
    cloud->points[i].z = inputCloud->points[i].z;
    cloud->points[i].intensity = inputCloud->points[i].ring;
  }
  
  downSizeFilter.setInputCloud(cloud);
  downSizeFilter.filter(*cloudDS);
  
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);
  
  vector<pcl::PCLPointField> fields;
  pcl::getFields<PointType>(fields);
  
  const int K = 5;
  outputCloud->points.clear();
  outputCloud->points.resize(cloudDS->points.size());
  for(long i = 0; i < cloudDS->points.size(); i++) {
    PointType newPnt;
    newPnt.x = cloudDS->points[i].x;
    newPnt.y = cloudDS->points[i].y;
    newPnt.z = cloudDS->points[i].z;
    newPnt.ring = cloudDS->points[i].intensity;
    vector<int> idxs(K);
    vector<float> sqrdDists(K);
    if(kdtree.nearestKSearch(cloudDS->points[i], K, idxs, sqrdDists) > 0) {
      float sums[fields.size() - 4];
      for(int j = 0; j < fields.size() - 4; j++) {
        sums[j] = 0;
      }
      for(int j = 0; j < idxs.size(); j++) {
        uint8_t *dataPnt = (uint8_t*)(inputCloud->points.data() + idxs[j]);
        for(int k = 4; k < fields.size(); k++) {
          sums[k - 4] += *((float*)(dataPnt + fields[k].offset));
        }
      }
      uint8_t *dataPnt = (uint8_t*)(&newPnt);
      for(int j = 4; j < fields.size(); j++) {
        *((float*)(dataPnt + fields[j].offset)) = sums[j - 4] / idxs.size();
      }
    } else {
      ROS_ERROR("No nearest points in KDTree!");
    }
    outputCloud->points[i] = newPnt;
  }
  cloud.reset();
  
  outputCloud->header = inputCloud->header;
  outputCloud->is_dense = cloudDS->is_dense;
  outputCloud->width = cloudDS->width;
  outputCloud->height = cloudDS->height;
  cloudDS.reset();
}

struct smoothness_t{ 
  float value;
  size_t ind;
};

struct by_value{ 
  bool operator()(smoothness_t const &left, smoothness_t const &right) { 
    return left.value < right.value;
  }
};

/*
  * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
  */
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  float ring;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                  (float, x, x) (float, y, y)
                                  (float, z, z) (float, ring, ring)
                                  (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                  (double, time, time)
)

typedef PointXYZIRPYT  PointTypePose;

#endif
