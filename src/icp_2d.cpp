/*
 * Copyright 2015 Fetch Robotics Inc.
 * Ported to ROS1.
 */

#include <lidar_auto_docking/icp_2d.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/SVD>

namespace icp_2d {

double thetaFromQuaternion(const geometry_msgs::Quaternion& q) {
  if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0)
    return 0.0;
  return 2.0 * atan2(q.z, q.w);
}

std::vector<geometry_msgs::Point> transform(
    const std::vector<geometry_msgs::Point>& points, double x, double y,
    double theta) {
  std::vector<geometry_msgs::Point> points_t;
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (size_t i = 0; i < points.size(); i++) {
    geometry_msgs::Point pt = points[i];
    geometry_msgs::Point pt_t;
    pt_t.x = cos_th * pt.x - sin_th * pt.y + x;
    pt_t.y = sin_th * pt.x + cos_th * pt.y + y;
    points_t.push_back(pt_t);
  }
  return points_t;
}

geometry_msgs::Point getCentroid(
    const std::vector<geometry_msgs::Point> points) {
  geometry_msgs::Point pt;
  for (size_t i = 0; i < points.size(); i++) {
    pt.x += points[i].x;
    pt.y += points[i].y;
  }
  pt.x /= points.size();
  pt.y /= points.size();
  return pt;
}

bool computeCorrespondences(
    const std::vector<geometry_msgs::Point>& source,
    const std::vector<geometry_msgs::Point>& target,
    std::vector<geometry_msgs::Point>& correspondences) {
  correspondences.clear();
  for (size_t i = 0; i < source.size(); i++) {
    double d = 1.0;
    size_t best = 0;
    for (size_t j = 0; j < target.size(); j++) {
      double dx = source[i].x - target[j].x;
      double dy = source[i].y - target[j].y;
      double dist = dx * dx + dy * dy;
      if (dist < d) { best = j; d = dist; }
    }
    if (d >= 1.0) return false;
    correspondences.push_back(target[best]);
  }
  return true;
}

bool alignPCA(const std::vector<geometry_msgs::Point> source,
              const std::vector<geometry_msgs::Point> target,
              geometry_msgs::Transform& t) {
  double theta = thetaFromQuaternion(t.rotation);
  std::vector<geometry_msgs::Point> source_t =
      transform(source, t.translation.x, t.translation.y, theta);

  geometry_msgs::Point cs = getCentroid(source_t);
  geometry_msgs::Point ct = getCentroid(target);

  t.translation.x += ct.x - cs.x;
  t.translation.y += ct.y - cs.y;

  Eigen::MatrixXf Ps(2, source_t.size());
  for (size_t i = 0; i < source_t.size(); i++) {
    Ps(0, i) = source_t[i].x - cs.x;
    Ps(1, i) = source_t[i].y - cs.y;
  }
  Eigen::MatrixXf Ms = Ps * Ps.transpose();
  Eigen::EigenSolver<Eigen::MatrixXf> solver_s(Ms);
  Eigen::MatrixXf A = solver_s.eigenvectors().real();
  Eigen::MatrixXf eig_of_Ms = solver_s.eigenvalues().real();
  if (eig_of_Ms(0, 0) < eig_of_Ms(1, 0)) {
    A.col(0).swap(A.col(1));
    A.col(1) = -A.col(1);
  }

  Eigen::MatrixXf Pt(2, target.size());
  for (size_t i = 0; i < target.size(); i++) {
    Pt(0, i) = target[i].x - ct.x;
    Pt(1, i) = target[i].y - ct.y;
  }
  Eigen::MatrixXf Mt = Pt * Pt.transpose();
  Eigen::EigenSolver<Eigen::MatrixXf> solver_t(Mt);
  Eigen::MatrixXf B = solver_t.eigenvectors().real();
  Eigen::MatrixXf eig_of_Mt = solver_t.eigenvalues().real();
  if (eig_of_Mt(0, 0) < eig_of_Mt(1, 0)) {
    B.col(0).swap(B.col(1));
    B.col(1) = -B.col(1);
  }

  Eigen::MatrixXf R = B * A.transpose();
  theta += atan2(R(1, 0), R(0, 0));
  t.rotation.z = sin(theta / 2.0);
  t.rotation.w = cos(theta / 2.0);
  return true;
}

bool alignSVD(const std::vector<geometry_msgs::Point> source,
              const std::vector<geometry_msgs::Point> target,
              geometry_msgs::Transform& t) {
  double theta = thetaFromQuaternion(t.rotation);
  std::vector<geometry_msgs::Point> source_t =
      transform(source, t.translation.x, t.translation.y, theta);

  std::vector<geometry_msgs::Point> corr;
  if (!computeCorrespondences(target, source_t, corr)) return false;

  geometry_msgs::Point cs = getCentroid(corr);
  geometry_msgs::Point ct = getCentroid(target);

  Eigen::MatrixXf P(2, corr.size());
  for (size_t i = 0; i < corr.size(); i++) {
    P(0, i) = corr[i].x - cs.x;
    P(1, i) = corr[i].y - cs.y;
  }
  Eigen::MatrixXf Q(2, target.size());
  for (size_t i = 0; i < target.size(); i++) {
    Q(0, i) = target[i].x - ct.x;
    Q(1, i) = target[i].y - ct.y;
  }

  Eigen::MatrixXf M = P * Q.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(
      M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXf R = svd.matrixV() * svd.matrixU().transpose();

  t.translation.x += (ct.x - cs.x);
  t.translation.y += (ct.y - cs.y);
  theta += atan2(R(1, 0), R(0, 0));
  t.rotation.z = sin(theta / 2.0);
  t.rotation.w = cos(theta / 2.0);
  return true;
}

double getRMSD(const std::vector<geometry_msgs::Point> source,
               const std::vector<geometry_msgs::Point> target) {
  std::vector<geometry_msgs::Point> corr;
  if (!computeCorrespondences(source, target, corr)) return 10e6;
  double rmsd = 0.0;
  for (size_t i = 0; i < source.size(); i++) {
    double dx = source[i].x - corr[i].x;
    double dy = source[i].y - corr[i].y;
    rmsd += dx * dx + dy * dy;
  }
  rmsd /= corr.size();
  return sqrt(rmsd);
}

double alignICP(const std::vector<geometry_msgs::Point> source,
                const std::vector<geometry_msgs::Point> target,
                geometry_msgs::Transform& t, size_t max_iterations,
                double min_delta_rmsd) {
  alignPCA(source, target, t);
  double prev_rmsd = -1.0;
  for (size_t iteration = 0; iteration < max_iterations; iteration++) {
    if (!alignSVD(source, target, t)) return -1.0;
    std::vector<geometry_msgs::Point> source_t =
        transform(source, t.translation.x, t.translation.y,
                  thetaFromQuaternion(t.rotation));
    double rmsd = getRMSD(target, source_t);
    if (prev_rmsd > 0.0 && fabs(prev_rmsd - rmsd) < min_delta_rmsd)
      return getRMSD(source_t, target);
    prev_rmsd = rmsd;
  }
  std::vector<geometry_msgs::Point> source_t =
      transform(source, t.translation.x, t.translation.y,
                thetaFromQuaternion(t.rotation));
  return getRMSD(source_t, target);
}

}  // namespace icp_2d
