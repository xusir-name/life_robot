#include "graspnet_pick/geometry.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace graspnet_pick {
namespace {

struct GridKey {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridKeyHash {
  std::size_t operator()(const GridKey &key) const {
    std::size_t h = 1469598103934665603ULL;
    auto mix = [&h](int value) {
      h ^= static_cast<std::size_t>(value);
      h *= 1099511628211ULL;
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return h;
  }
};

GridKey cellOf(const Eigen::Vector3d &point, double cell_size) {
  return GridKey{static_cast<int>(std::floor(point.x() / cell_size)),
                 static_cast<int>(std::floor(point.y() / cell_size)),
                 static_cast<int>(std::floor(point.z() / cell_size))};
}

double cross2d(const Eigen::Vector2d &o, const Eigen::Vector2d &a,
               const Eigen::Vector2d &b) {
  return (a.x() - o.x()) * (b.y() - o.y()) -
         (a.y() - o.y()) * (b.x() - o.x());
}

std::vector<Eigen::Vector2d> convexHull2d(std::vector<Eigen::Vector2d> points) {
  std::sort(points.begin(), points.end(), [](const auto &a, const auto &b) {
    if (a.x() != b.x()) {
      return a.x() < b.x();
    }
    return a.y() < b.y();
  });
  points.erase(std::unique(points.begin(), points.end(),
                           [](const auto &a, const auto &b) {
                             return (a - b).norm() < 1e-12;
                           }),
               points.end());
  if (points.size() <= 2) {
    return points;
  }

  std::vector<Eigen::Vector2d> lower;
  for (const auto &p : points) {
    while (lower.size() >= 2 &&
           cross2d(lower[lower.size() - 2], lower.back(), p) <= 0.0) {
      lower.pop_back();
    }
    lower.push_back(p);
  }
  std::vector<Eigen::Vector2d> upper;
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    while (upper.size() >= 2 &&
           cross2d(upper[upper.size() - 2], upper.back(), *it) <= 0.0) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }
  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  return lower;
}

struct Rect2D {
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  Eigen::Vector2d u{Eigen::Vector2d(1.0, 0.0)};
  Eigen::Vector2d v{Eigen::Vector2d(0.0, 1.0)};
  double extent_u{0.0};
  double extent_v{0.0};
};

Rect2D minAreaRect2d(const std::vector<Eigen::Vector2d> &points) {
  const auto hull = convexHull2d(points);
  if (hull.size() < 2) {
    return Rect2D{};
  }

  bool found = false;
  double best_area = std::numeric_limits<double>::infinity();
  Rect2D best;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Eigen::Vector2d edge = hull[(i + 1) % hull.size()] - hull[i];
    const double norm = edge.norm();
    if (norm < 1e-9) {
      continue;
    }
    const Eigen::Vector2d u = edge / norm;
    const Eigen::Vector2d v(-u.y(), u.x());
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const auto &p : hull) {
      const double pu = p.dot(u);
      const double pv = p.dot(v);
      min_u = std::min(min_u, pu);
      max_u = std::max(max_u, pu);
      min_v = std::min(min_v, pv);
      max_v = std::max(max_v, pv);
    }
    const double area = (max_u - min_u) * (max_v - min_v);
    if (!found || area < best_area) {
      found = true;
      best_area = area;
      best.u = u;
      best.v = v;
      best.center = u * ((min_u + max_u) * 0.5) + v * ((min_v + max_v) * 0.5);
      best.extent_u = max_u - min_u;
      best.extent_v = max_v - min_v;
    }
  }
  return best;
}

std::optional<Grasp> makeGrasp(const Eigen::Vector3d &approach,
                               const Eigen::Vector3d &closing,
                               const Eigen::Vector3d &center, double width_m,
                               double score, int cluster_id) {
  Eigen::Vector3d z = approach;
  if (z.norm() < 1e-9) {
    return std::nullopt;
  }
  z.normalize();

  Eigen::Vector3d y = closing;
  y -= y.dot(z) * z;
  if (y.norm() < 1e-9) {
    return std::nullopt;
  }
  y.normalize();

  Eigen::Vector3d x = y.cross(z);
  if (x.norm() < 1e-9) {
    return std::nullopt;
  }
  x.normalize();
  y = z.cross(x);
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  if (R.determinant() < 0.0) {
    R.col(0) *= -1.0;
  }

  Grasp grasp;
  grasp.score = score;
  grasp.width_m = width_m;
  grasp.position = center;
  grasp.rotation = R;
  grasp.cluster_id = cluster_id;
  return grasp;
}


}  // namespace

PointCloud depthToPoints(const cv::Mat &depth, const Eigen::Matrix3d &K,
                         const std::optional<cv::Rect> &roi, int stride,
                         double min_depth_m, double max_depth_m) {
  PointCloud out;
  if (depth.empty()) {
    return out;
  }
  int x1 = 0;
  int y1 = 0;
  int x2 = depth.cols;
  int y2 = depth.rows;
  if (roi.has_value()) {
    x1 = std::max(0, roi->x);
    y1 = std::max(0, roi->y);
    x2 = std::min(depth.cols, roi->x + roi->width);
    y2 = std::min(depth.rows, roi->y + roi->height);
  }
  if (x2 <= x1 || y2 <= y1) {
    return out;
  }
  stride = std::max(1, stride);

  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);
  const int type = depth.type();
  for (int v = y1; v < y2; v += stride) {
    for (int u = x1; u < x2; u += stride) {
      double z = 0.0;
      if (type == CV_32F) {
        z = depth.at<float>(v, u);
      } else if (type == CV_64F) {
        z = depth.at<double>(v, u);
      } else if (type == CV_16U) {
        z = static_cast<double>(depth.at<std::uint16_t>(v, u));
      } else {
        throw std::invalid_argument("depthToPoints: unsupported cv::Mat type");
      }
      if (!std::isfinite(z) || z <= min_depth_m || z >= max_depth_m) {
        continue;
      }
      out.points.emplace_back((u - cx) * z / fx, (v - cy) * z / fy, z);
      out.uv.emplace_back(u, v);
    }
  }
  return out;
}

std::vector<Eigen::Vector3d> voxelDownsample(
    const std::vector<Eigen::Vector3d> &points, double voxel_m) {
  if (voxel_m <= 0.0 || points.empty()) {
    return points;
  }
  struct Accum {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    int count{0};
  };
  std::unordered_map<GridKey, Accum, GridKeyHash> voxels;
  for (const auto &p : points) {
    auto &acc = voxels[cellOf(p, voxel_m)];
    acc.sum += p;
    ++acc.count;
  }
  std::vector<Eigen::Vector3d> out;
  out.reserve(voxels.size());
  for (const auto &entry : voxels) {
    out.push_back(entry.second.sum / static_cast<double>(entry.second.count));
  }
  return out;
}

PlaneResult removeDominantPlane(
    const std::vector<Eigen::Vector3d> &points, double distance_threshold_m,
    double min_inlier_ratio, int min_plane_points, int iterations, int seed,
    const std::optional<Eigen::Vector3d> &expected_normal,
    double max_normal_angle_rad) {
  PlaneResult out;
  if (static_cast<int>(points.size()) < std::max(50, min_plane_points)) {
    out.points = points;
    return out;
  }

  std::optional<Eigen::Vector3d> expected;
  if (expected_normal.has_value() && expected_normal->norm() > 1e-9) {
    expected = expected_normal->normalized();
  }
  const double cos_limit = std::cos(max_normal_angle_rad);
  std::mt19937_64 rng(static_cast<std::mt19937_64::result_type>(seed));
  std::uniform_int_distribution<int> dist(0, static_cast<int>(points.size()) - 1);

  std::vector<int> best_inliers;
  Plane best_plane;
  bool found = false;
  for (int iter = 0; iter < std::max(1, iterations); ++iter) {
    int i0 = dist(rng);
    int i1 = dist(rng);
    int i2 = dist(rng);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }
    const Eigen::Vector3d normal =
        (points[i1] - points[i0]).cross(points[i2] - points[i0]);
    if (normal.norm() < 1e-9) {
      continue;
    }
    const Eigen::Vector3d n = normal.normalized();
    if (expected.has_value() &&
        std::abs(n.dot(*expected)) < cos_limit) {
      continue;
    }
    const double offset = -n.dot(points[i0]);
    std::vector<int> inliers;
    inliers.reserve(points.size() / 4);
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (std::abs(points[i].dot(n) + offset) < distance_threshold_m) {
        inliers.push_back(static_cast<int>(i));
      }
    }
    if (!found || inliers.size() > best_inliers.size()) {
      found = true;
      best_inliers = std::move(inliers);
      best_plane.normal = n;
      best_plane.offset = offset;
    }
  }

  const int n = static_cast<int>(points.size());
  if (!found ||
      static_cast<int>(best_inliers.size()) <
          std::max(min_plane_points,
                   static_cast<int>(min_inlier_ratio * static_cast<double>(n)))) {
    out.points = points;
    return out;
  }
  std::vector<bool> keep(points.size(), true);
  for (int index : best_inliers) {
    keep[static_cast<std::size_t>(index)] = false;
  }
  out.points.reserve(points.size() - best_inliers.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (keep[i]) {
      out.points.push_back(points[i]);
    }
  }
  out.plane = best_plane;
  return out;
}

std::vector<int> clusterPoints(const std::vector<Eigen::Vector3d> &points,
                               double eps_m, int min_points) {
  const int n = static_cast<int>(points.size());
  std::vector<int> labels(static_cast<std::size_t>(n), -1);
  if (n == 0) {
    return labels;
  }
  if (eps_m <= 0.0) {
    std::fill(labels.begin(), labels.end(), 0);
    return labels;
  }

  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
  for (int i = 0; i < n; ++i) {
    grid[cellOf(points[static_cast<std::size_t>(i)], eps_m)].push_back(i);
  }

  int next_label = 0;
  const double eps2 = eps_m * eps_m;
  for (int i = 0; i < n; ++i) {
    if (labels[static_cast<std::size_t>(i)] != -1) {
      continue;
    }
    labels[static_cast<std::size_t>(i)] = next_label;
    std::vector<int> stack{i};
    std::vector<int> members;
    while (!stack.empty()) {
      const int j = stack.back();
      stack.pop_back();
      members.push_back(j);
      const GridKey key = cellOf(points[static_cast<std::size_t>(j)], eps_m);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto it = grid.find(GridKey{key.x + dx, key.y + dy, key.z + dz});
            if (it == grid.end()) {
              continue;
            }
            for (int k : it->second) {
              if (labels[static_cast<std::size_t>(k)] != -1) {
                continue;
              }
              if ((points[static_cast<std::size_t>(k)] -
                   points[static_cast<std::size_t>(j)])
                      .squaredNorm() <= eps2) {
                labels[static_cast<std::size_t>(k)] = next_label;
                stack.push_back(k);
              }
            }
          }
        }
      }
    }
    if (static_cast<int>(members.size()) < min_points) {
      for (int member : members) {
        labels[static_cast<std::size_t>(member)] = -1;
      }
    } else {
      ++next_label;
    }
  }
  return labels;
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> pcaObb(
    const std::vector<Eigen::Vector3d> &points) {
  if (points.empty()) {
    return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
            Eigen::Vector3d::Zero()};
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto &p : points) {
    mean += p;
  }
  mean /= static_cast<double>(points.size());

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto &p : points) {
    const Eigen::Vector3d d = p - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(std::max<std::size_t>(points.size(), 1));

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  Eigen::Matrix3d axes = solver.eigenvectors();
  // SelfAdjointEigenSolver returns ascending eigenvalues; reverse to descending.
  axes.col(0).swap(axes.col(2));
  if (axes.determinant() < 0.0) {
    axes.col(2) *= -1.0;
  }

  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d hi = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto &p : points) {
    const Eigen::Vector3d proj = axes.transpose() * (p - mean);
    lo = lo.cwiseMin(proj);
    hi = hi.cwiseMax(proj);
  }
  const Eigen::Vector3d center = mean + axes * ((lo + hi) * 0.5);
  const Eigen::Vector3d extents = (hi - lo).cwiseMax(1e-6);
  return {center, axes, extents};
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d> tabletopObb(
    const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &up) {
  if (points.empty() || up.norm() < 1e-9) {
    return pcaObb(points);
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto &p : points) {
    mean += p;
  }
  mean /= static_cast<double>(points.size());

  Eigen::Vector3d up_n = up.normalized();
  const Eigen::Vector3d to_camera = -mean / std::max(mean.norm(), 1e-9);
  if (up_n.dot(to_camera) < 0.0) {
    up_n = -up_n;
  }

  Eigen::Vector3d helper(1.0, 0.0, 0.0);
  if (std::abs(up_n.dot(helper)) > 0.9) {
    helper = Eigen::Vector3d(0.0, 1.0, 0.0);
  }
  Eigen::Vector3d e1 = up_n.cross(helper);
  e1.normalize();
  const Eigen::Vector3d e2 = up_n.cross(e1);

  std::vector<Eigen::Vector2d> proj2;
  proj2.reserve(points.size());
  for (const auto &p : points) {
    proj2.emplace_back(p.dot(e1), p.dot(e2));
  }
  const Rect2D rect = minAreaRect2d(proj2);
  const Eigen::Vector3d u3 = e1 * rect.u.x() + e2 * rect.u.y();
  const Eigen::Vector3d v3 = e1 * rect.v.x() + e2 * rect.v.y();
  Eigen::Matrix3d axes;
  axes.col(0) = u3;
  axes.col(1) = v3;
  axes.col(2) = up_n;
  if (axes.determinant() < 0.0) {
    axes.col(1) *= -1.0;
  }

  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d hi = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const auto &p : points) {
    const Eigen::Vector3d proj = axes.transpose() * p;
    lo = lo.cwiseMin(proj);
    hi = hi.cwiseMax(proj);
  }
  const Eigen::Vector3d center = axes * ((lo + hi) * 0.5);
  const Eigen::Vector3d extents = (hi - lo).cwiseMax(1e-6);
  return {center, axes, extents};
}

std::vector<Grasp> nmsGrasps(std::vector<Grasp> grasps, double translation_m,
                             double rotation_rad, int max_grasps) {
  std::sort(grasps.begin(), grasps.end(),
            [](const Grasp &a, const Grasp &b) { return a.score > b.score; });
  std::vector<Grasp> kept;
  for (const auto &grasp : grasps) {
    bool duplicate = false;
    for (const auto &k : kept) {
      if ((grasp.position - k.position).norm() < translation_m &&
          rotationAngleRad(grasp.rotation, k.rotation) < rotation_rad) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      kept.push_back(grasp);
    }
    if (static_cast<int>(kept.size()) >= max_grasps) {
      break;
    }
  }
  return kept;
}


std::vector<Grasp> sampleGrasps(const std::vector<Eigen::Vector3d> &points,
                                const GraspSamplingOptions &options) {
  if (static_cast<int>(points.size()) < options.cluster_min_points) {
    return {};
  }
  if (!(options.min_width_m < options.max_width_m)) {
    throw std::invalid_argument("min_width_m must be smaller than max_width_m");
  }
  const double preferred =
      std::clamp(options.preferred_width_m, options.min_width_m, options.max_width_m);
  const double width_span =
      std::max(options.max_width_m - options.min_width_m, 1e-6);

  std::optional<Eigen::Vector3d> up_hint;
  if (options.up.has_value() && options.up->norm() > 1e-9) {
    up_hint = options.up->normalized();
  }

  const std::vector<int> labels =
      clusterPoints(points, options.cluster_eps_m, options.cluster_min_points);
  std::set<int> unique_labels;
  for (int label : labels) {
    if (label >= 0) {
      unique_labels.insert(label);
    }
  }

  std::vector<Grasp> candidates;
  for (int label : unique_labels) {
    std::vector<Eigen::Vector3d> cluster;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (labels[i] == label) {
        cluster.push_back(points[i]);
      }
    }
    if (static_cast<int>(cluster.size()) < options.cluster_min_points) {
      continue;
    }

    Eigen::Vector3d center;
    Eigen::Matrix3d axes;
    Eigen::Vector3d extents;
    if (options.up.has_value()) {
      std::tie(center, axes, extents) = tabletopObb(cluster, *options.up);
    } else {
      std::tie(center, axes, extents) = pcaObb(cluster);
    }
    const double radius = center.norm();
    if (radius < 1e-6) {
      continue;
    }
    const Eigen::Vector3d to_object = center / radius;

    std::optional<Eigen::Vector3d> up_local = up_hint;
    if (up_local.has_value() && up_local->dot(to_object) > 0.0) {
      *up_local = -*up_local;
    }

    std::vector<Eigen::Vector3d> approach_axes{
        axes.col(0), axes.col(1), axes.col(2), to_object};
    for (const auto &approach : approach_axes) {
      Eigen::Vector3d a = approach;
      if (a.dot(to_object) < 0.0) {
        a = -a;
      }
      for (int close_index = 0; close_index < 3; ++close_index) {
        const double width = extents(close_index);
        if (width < options.min_width_m || width > options.max_width_m) {
          continue;
        }
        const double width_score =
            std::max(0.0, 1.0 - std::abs(width - preferred) / width_span);
        const double approach_score = std::max(0.0, a.dot(to_object));
        const double vertical_score =
            up_local.has_value() ? std::max(0.0, a.dot(-*up_local)) : 0.0;
        const double score = 0.55 * width_score + 0.35 * approach_score +
                             0.10 * vertical_score;
        auto grasp = makeGrasp(a, axes.col(close_index), center, width, score, label);
        if (grasp.has_value()) {
          candidates.push_back(*grasp);
        }
      }
    }
  }

  return nmsGrasps(std::move(candidates), options.nms_translation_m,
                   options.nms_rotation_rad, options.max_grasps);
}

std::optional<Grasp> selectBestGrasp(const std::vector<Grasp> &grasps,
                                     double min_score) {
  std::optional<Grasp> best;
  for (const auto &grasp : grasps) {
    if (grasp.score < min_score) {
      continue;
    }
    if (!best.has_value() || grasp.score > best->score) {
      best = grasp;
    }
  }
  return best;
}

double rotationAngleRad(const Eigen::Matrix3d &R1, const Eigen::Matrix3d &R2) {
  const double cos_value = (R1.transpose() * R2).trace() - 1.0;
  return std::acos(std::clamp(cos_value * 0.5, -1.0, 1.0));
}

}  // namespace graspnet_pick
