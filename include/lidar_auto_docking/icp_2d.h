#ifndef LIDAR_AUTO_DOCKING_ICP_2D_H
#define LIDAR_AUTO_DOCKING_ICP_2D_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <vector>

namespace icp_2d {

double thetaFromQuaternion(const geometry_msgs::Quaternion& q);

std::vector<geometry_msgs::Point> transform(
    const std::vector<geometry_msgs::Point>& points, double x, double y,
    double theta);

geometry_msgs::Point getCentroid(
    const std::vector<geometry_msgs::Point> points);

bool alignPCA(const std::vector<geometry_msgs::Point> source,
              const std::vector<geometry_msgs::Point> target,
              geometry_msgs::Transform& transform);

bool alignSVD(const std::vector<geometry_msgs::Point> source,
              const std::vector<geometry_msgs::Point> target,
              geometry_msgs::Transform& transform);

double alignICP(const std::vector<geometry_msgs::Point> source,
                const std::vector<geometry_msgs::Point> target,
                geometry_msgs::Transform& transform,
                size_t max_iterations = 10, double min_delta_rmsd = 0.000001);

}  // namespace icp_2d

#endif  // LIDAR_AUTO_DOCKING_ICP_2D_H
