#ifndef __MATCH_POINTS_H__
#define __MATCH_POINTS_H__

#include "search_tree/kd_tree.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <vector>

/*void match_points(const pcl::PointCloud<pcl::PointXY>::Ptr &sourcePtr, const
pcl::PointCloud<pcl::PointXY>::Ptr &targetPtr, double cutoff_sq)
{
    // std::vector<bool> source_used(sourcePtr->);
    // std::vector<bool> target_used(targetPtr.);
}*/

struct mapping_data {};

float find_mapping_distance(const pcl::PointCloud<pcl::PointXY>::Ptr &cloud) {
  std::vector<pcl::PointXY> points;

  if (cloud->size() < 2) {
    return -1.f;
  }

  for (size_t i_p = 0; i_p < cloud->size(); i_p++) {
    points.push_back(cloud->at(i_p));
  }
  size_t num_points = points.size();

  float total_sum_dist = 0.0;
  size_t total_num_dist = 0;
  std::vector<float> min_dist_sq(num_points, std::numeric_limits<float>::max());

  for (size_t i_p = 0; i_p < num_points; i_p++) {
    for (size_t j_p = i_p + 1; j_p < num_points; j_p++) {
      float dist_sq = pcl::squaredEuclideanDistance(points[i_p], points[j_p]);
      min_dist_sq[i_p] = std::min(min_dist_sq[i_p], dist_sq);
      min_dist_sq[j_p] = std::min(min_dist_sq[j_p], dist_sq);
    }
  }

  std::sort(min_dist_sq.begin(), min_dist_sq.end());

  std::cerr << "Min neighbor sq dist:\t" << min_dist_sq[0] << std::endl;
  std::cerr << "Max neighbor sq dist:\t" << min_dist_sq[num_points - 1]
            << std::endl;

  return std::sqrt(min_dist_sq[num_points / 2]);
}

#endif