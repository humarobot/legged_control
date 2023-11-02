#pragma once
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <vector>
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using VectorXd = Eigen::VectorXd;

struct KnotPoint{
  Vector3d position{Vector3d::Zero()};
  Vector3d velocity{Vector3d::Zero()};
};

class HermiteSpline{
public:
  HermiteSpline(const std::vector<KnotPoint>& knots,const double t_max);
  HermiteSpline() = default;
  ~HermiteSpline() = default;
  void SetKnotsTMax(const std::vector<KnotPoint>& knots,const double t_max);
  Vector3d getPosition(double t) const;
  Vector3d getVelocity(double t) const;
  Vector3d getAcceleration(double t) const;
private:
  double h00(double t) const { return 2*t*t*t - 3*t*t + 1; }
  double h10(double t) const { return t*t*t - 2*t*t + t; }
  double h01(double t) const { return -2*t*t*t + 3*t*t; }
  double h11(double t) const { return t*t*t - t*t; }
  double h00d(double t) const { return 6*t*t - 6*t; }
  double h10d(double t) const { return 3*t*t - 4*t + 1; }
  double h01d(double t) const { return -6*t*t + 6*t; }
  double h11d(double t) const { return 3*t*t - 2*t; }

  std::vector<KnotPoint> knots_{1,KnotPoint{}};
  int num_knots_{1};
  int num_segments_{0};
  double t_max_{0.0};
  VectorXd x_knots_{VectorXd::Zero(1)};
};