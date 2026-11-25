#include "graspnet_pick/transforms.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace graspnet_pick {

Eigen::Matrix4d transformFromQuat(const Eigen::Vector3d &translation,
                                  const Eigen::Quaterniond &rotation) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = translation;
  return T;
}

Eigen::Matrix4d invertTransform(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  out.block<3, 3>(0, 0) = R.transpose();
  out.block<3, 1>(0, 3) = -R.transpose() * t;
  return out;
}

Eigen::Matrix4d translationTransform(double x, double y, double z) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}

Eigen::Quaterniond matrixToQuat(const Eigen::Matrix3d &R) {
  Eigen::Quaterniond q(R);
  if (q.norm() < 1e-12) {
    throw std::runtime_error("invalid rotation matrix: zero quaternion");
  }
  q.normalize();
  return q;
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> matrixToPose(const Eigen::Matrix4d &T) {
  return {T.block<3, 1>(0, 3), matrixToQuat(T.block<3, 3>(0, 0))};
}

Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::Pose &pose) {
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                       pose.orientation.z);
  if (q.norm() < 1e-12) {
    throw std::invalid_argument("pose has zero quaternion");
  }
  q.normalize();
  return transformFromQuat(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z), q);
}

void validateTransform(const Eigen::Matrix4d &T, const std::string &name) {
  if (!T.allFinite()) {
    throw std::invalid_argument(name + ": transform has non-finite values");
  }
  if (std::abs(T(3, 0)) > 1e-6 || std::abs(T(3, 1)) > 1e-6 ||
      std::abs(T(3, 2)) > 1e-6 || std::abs(T(3, 3) - 1.0) > 1e-6) {
    throw std::invalid_argument(name + ": last row must be [0, 0, 0, 1]");
  }
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  if (!(R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-5)) {
    throw std::invalid_argument(name + ": rotation is not orthonormal");
  }
  if (R.determinant() < 0.0) {
    throw std::invalid_argument(name + ": rotation determinant is negative");
  }
}

namespace {

Eigen::Matrix4d matrixFromNode(const YAML::Node &node, const std::string &name) {
  if (!node.IsSequence() || node.size() != 4) {
    throw std::invalid_argument(name + ": expected 4x4 sequence");
  }
  Eigen::Matrix4d T;
  for (int r = 0; r < 4; ++r) {
    const YAML::Node row = node[r];
    if (!row.IsSequence() || row.size() != 4) {
      throw std::invalid_argument(name + ": expected 4 columns per row");
    }
    for (int c = 0; c < 4; ++c) {
      T(r, c) = row[c].as<double>();
    }
  }
  return T;
}

Eigen::Vector3d vector3FromNode(const YAML::Node &node, const std::string &name) {
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(name + ": expected 3 values");
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(),
                         node[2].as<double>());
}

}  // namespace

Extrinsics loadExtrinsicsYaml(const std::string &path) {
  std::string expanded_path = path;
  if (expanded_path.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (home != nullptr) {
      expanded_path = std::string(home) + expanded_path.substr(1);
    }
  }
  YAML::Node data;
  try {
    data = YAML::LoadFile(expanded_path);
  } catch (const YAML::Exception &exc) {
    throw std::runtime_error("failed to load " + expanded_path + ": " + exc.what());
  }
  if (!data.IsMap()) {
    throw std::invalid_argument(expanded_path + ": expected a YAML mapping");
  }

  Extrinsics out;
  if (data["transform"]) {
    out.T_tcp_cam = matrixFromNode(data["transform"], path + ":transform");
  } else if (data["transform_tcp_camera"]) {
    out.T_tcp_cam =
        matrixFromNode(data["transform_tcp_camera"], path + ":transform_tcp_camera");
  } else if (data["translation_m"]) {
    const Eigen::Vector3d t = vector3FromNode(data["translation_m"], path + ":translation_m");
    if (data["rotation_quaternion_xyzw"]) {
      const YAML::Node qn = data["rotation_quaternion_xyzw"];
      if (!qn.IsSequence() || qn.size() != 4) {
        throw std::invalid_argument(path + ": rotation_quaternion_xyzw must have 4 values");
      }
      Eigen::Quaterniond q(qn[3].as<double>(), qn[0].as<double>(),
                           qn[1].as<double>(), qn[2].as<double>());
      out.T_tcp_cam = transformFromQuat(t, q);
    } else if (data["rotation_matrix"]) {
      const YAML::Node rn = data["rotation_matrix"];
      if (!rn.IsSequence() || rn.size() != 3) {
        throw std::invalid_argument(path + ": rotation_matrix must be 3x3");
      }
      Eigen::Matrix3d R;
      for (int r = 0; r < 3; ++r) {
        if (!rn[r].IsSequence() || rn[r].size() != 3) {
          throw std::invalid_argument(path + ": rotation_matrix must be 3x3");
        }
        for (int c = 0; c < 3; ++c) {
          R(r, c) = rn[r][c].as<double>();
        }
      }
      out.T_tcp_cam = Eigen::Matrix4d::Identity();
      out.T_tcp_cam.block<3, 3>(0, 0) = R;
      out.T_tcp_cam.block<3, 1>(0, 3) = t;
    } else {
      throw std::invalid_argument(path + ": missing rotation");
    }
  } else {
    throw std::invalid_argument(path + ": no transform / translation_m found");
  }

  validateTransform(out.T_tcp_cam, path);
  if (data["parent_frame"]) {
    out.parent_frame = data["parent_frame"].as<std::string>();
  }
  if (data["child_frame"]) {
    out.child_frame = data["child_frame"].as<std::string>();
  }
  return out;
}

}  // namespace graspnet_pick
