// include/eigen_types.hpp
#pragma once

#include <Eigen/Dense>

// 手动定义固定尺寸向量（Eigen 不自带 Vector6d / Vector7d）
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector3d = Eigen::Vector3d;  // 这个是自带的，可用