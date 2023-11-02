#include "HermiteSpline.hpp"

HermiteSpline::HermiteSpline(const std::vector<KnotPoint>& knots, const double t_max) : t_max_(t_max) {
  knots_ = knots;
  num_knots_ = knots_.size();
  num_segments_ = num_knots_ - 1;
  // Equally spaced knots
  x_knots_ = VectorXd::Zero(num_knots_);
  for (int i = 0; i < num_knots_; ++i) {
    x_knots_(i) = i * t_max_ / num_segments_;
  }
}

void HermiteSpline::SetKnotsTMax(const std::vector<KnotPoint>& knots, const double t_max) {
  t_max_ = t_max;
  knots_ = knots;
  num_knots_ = knots_.size();
  num_segments_ = num_knots_ - 1;
  // Equally spaced knots
  x_knots_ = VectorXd::Zero(num_knots_);
  for (int i = 0; i < num_knots_; ++i) {
    x_knots_(i) = i * t_max_ / num_segments_;
  }
}

Vector3d HermiteSpline::getPosition(double t) const {
  Vector3d position{Vector3d::Zero()};
  if (t < 0.0) {
    position = knots_[0].position;
  } else if (t > t_max_) {
    position = knots_[num_knots_ - 1].position;
  } else {
    for (int i = 0; i < num_segments_; ++i) {
      if (t >= x_knots_(i) && t <= x_knots_(i + 1)) {
        auto h0 = h00((t - x_knots_(i)) / (x_knots_(i + 1) - x_knots_(i)));
        auto h1 = h10((t - x_knots_(i)) / (x_knots_(i + 1) - x_knots_(i)));
        auto h2 = h01((t - x_knots_(i)) / (x_knots_(i + 1) - x_knots_(i)));
        auto h3 = h11((t - x_knots_(i)) / (x_knots_(i + 1) - x_knots_(i)));
        position = h0 * knots_[i].position + h1 * (x_knots_(i + 1) - x_knots_(i)) * knots_[i].velocity +
                   h2 * knots_[i + 1].position + h3 * (x_knots_(i + 1) - x_knots_(i)) * knots_[i + 1].velocity;
        break;
      }
    }
  }
  return position;
}

Vector3d HermiteSpline::getVelocity(double t) const {
  Vector3d velocity{Vector3d::Zero()};
  if (t < 0.0) {
    velocity = knots_[0].velocity;
  } else if (t > t_max_) {
    velocity = knots_[num_knots_ - 1].velocity;
  } else {
    for (int i = 0; i < num_segments_; ++i) {
      if (t >= x_knots_(i) && t <= x_knots_(i + 1)) {
        t = (t-x_knots_(i))/(x_knots_(i+1)-x_knots_(i));
        velocity =
            (h00d(t) * knots_[i].position + h10d(t) * (x_knots_(i + 1) - x_knots_(i)) * knots_[i].velocity +
             h01d(t) * knots_[i + 1].position + h11d(t) * (x_knots_(i + 1) - x_knots_(i)) * knots_[i + 1].velocity) /
            (x_knots_(i + 1) - x_knots_(i));
        break;
      }
    }
  }
  return velocity;
}