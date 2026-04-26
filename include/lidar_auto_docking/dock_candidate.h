#ifndef LIDAR_AUTO_DOCKING_DOCK_CANDIDATE_H
#define LIDAR_AUTO_DOCKING_DOCK_CANDIDATE_H

#include <cmath>
#include <memory>
#include <vector>
#include <geometry_msgs/Point.h>

struct DockCandidate {
  std::vector<geometry_msgs::Point> points;
  double dist;

  double width() {
    if (points.empty()) return 0;
    geometry_msgs::Point& pt1 = points.front();
    geometry_msgs::Point& pt2 = points.back();
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) +
                     std::pow((pt1.y - pt2.y), 2));
  }

  bool valid(bool dock_found) {
    if (points.empty()) return false;
    if (width() > 0.5 || width() < 0.25) return false;
    if (dock_found) return dist < (0.25 * 0.25);
    return dist < 1.0;
  }
};

typedef std::shared_ptr<DockCandidate> DockCandidatePtr;

struct CompareCandidates {
  bool operator()(DockCandidatePtr a, DockCandidatePtr b) {
    return (a->dist > b->dist);
  }
};

#endif  // LIDAR_AUTO_DOCKING_DOCK_CANDIDATE_H
