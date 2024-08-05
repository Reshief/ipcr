#ifndef __MATCH_POINTS_H__
#define __MATCH_POINTS_H__

//#include "search_tree/kd_tree.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <queue>
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

  // Don't allow single points. At least 2 cells must be available.
  if (cloud->size() < 2) {
    std::cerr << "At least two cells must be included in the data" << std::endl;
    return -1.f;
  }

  for (size_t i_p = 0; i_p < cloud->size(); i_p++) {
    points.push_back(cloud->at(i_p));
  }
  size_t num_points = points.size();

  float total_sum_dist = 0.0;
  size_t total_num_dist = 0;
  std::vector<float> min_dist_sq(num_points, std::numeric_limits<float>::max());

  // Calculate the minimum neighbor distance of all cells
  for (size_t i_p = 0; i_p < num_points; i_p++) {
    for (size_t j_p = i_p + 1; j_p < num_points; j_p++) {
      float dist_sq = pcl::squaredEuclideanDistance(points[i_p], points[j_p]);
      min_dist_sq[i_p] = std::min(min_dist_sq[i_p], dist_sq);
      min_dist_sq[j_p] = std::min(min_dist_sq[j_p], dist_sq);
    }
  }

  // Sort distances to closest neighbor of each cell
  std::sort(min_dist_sq.begin(), min_dist_sq.end());

  // Some debug output
  std::cerr << "Min neighbor sq dist:\t" << min_dist_sq[0] << std::endl;
  std::cerr << "Max neighbor sq dist:\t" << min_dist_sq[num_points - 1]
            << std::endl;

  /* Use about half of the median distance as cutoff for the mapping
   * consideration
   */
  return std::sqrt(min_dist_sq[num_points / 2]) * 0.7;
}

std::pair<float, std::vector<std::pair<size_t, size_t>>>
find_mapping(const pcl::PointCloud<pcl::PointXY>::Ptr &clouda,
             const pcl::PointCloud<pcl::PointXY>::Ptr &cloudb,
             float cutoff = std::numeric_limits<float>::max()) {
  size_t before_count = clouda->size();
  size_t after_count = cloudb->size();

  std::vector<bool> before_matched(before_count, false);
  std::vector<bool> after_matched(after_count, false);
  typedef std::pair<float, std::pair<size_t, size_t>> mapping_entry;
  std::priority_queue<mapping_entry, std::vector<mapping_entry>,
                      std::greater<mapping_entry>>
      queue;

  std::vector<pcl::PointXY> pointsa, pointsb;

  for (size_t i_p = 0; i_p < before_count; i_p++) {
    pointsa.push_back(clouda->at(i_p));
  }

  for (size_t i_p = 0; i_p < after_count; i_p++) {
    pointsb.push_back(cloudb->at(i_p));
  }

  float sq_cutoff = cutoff * cutoff;

  // Calculate the minimum neighbor distance of all cells
  for (size_t i_p = 0; i_p < before_count; i_p++) {
    for (size_t j_p = 0; j_p < after_count; j_p++) {
      float dist_sq = pcl::squaredEuclideanDistance(pointsa[i_p], pointsb[j_p]);
      if (dist_sq < sq_cutoff) {
        queue.push({dist_sq, {i_p, j_p}});
      }
    }
  }

  std::vector<std::pair<size_t, size_t>> mapping;

  float total_err = 0.0f;

  for (; !queue.empty(); queue.pop()) {
    auto entry = queue.top();

    size_t before_p = entry.second.first;
    size_t after_p = entry.second.second;

    if (!before_matched[before_p] && !after_matched[after_p]) {
      before_matched[before_p] = true;
      after_matched[after_p] = true;
      mapping.push_back({before_p, after_p});
      total_err += std::sqrt(entry.first);
    }
  }

  return {total_err, mapping};
}

#endif