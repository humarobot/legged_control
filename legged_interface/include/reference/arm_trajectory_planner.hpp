#pragma once
#include <utility>

#include "ocs2_legged_robot/common/Types.h"
#include "queue"
#include "deque"

namespace ocs2
{
namespace legged_robot
{

struct TrapezoidalProfile
{
  scalar_t v{ 0.0 };    // max velocity // 0 means max velocity can't be reached
  scalar_t a{ 0.0 };    // max acceleration
  scalar_t T{ 0.0 };    // total time
  scalar_t T_s{ 0.0 };  // start time

  TrapezoidalProfile() = default;
  TrapezoidalProfile(scalar_t v, scalar_t a, scalar_t T, scalar_t T_s = 0.0) : v(v), a(a), T(T), T_s(T_s)
  {
  }
  scalar_t ddots(scalar_t t) const
  {
    // For max velocity can't be reached
    if (v == 0)
    {
      if (t < T_s)
        return 0;
      else if (t < T_s + T / 2.0)
        return a;
      else if (t <= T_s + T)
        return -a;
      else
        return 0;
    }
    // For max velocity can be reached
    else
    {
      if (t < T_s)
        return 0;
      else if (t < T_s + v / a)
        return a;
      else if (t < T_s + T - v / a)
        return 0;
      else if (t <= T_s + T)
        return -a;
      else
        return 0;
    }
  }
  scalar_t dots(scalar_t t) const
  {
    // For max velocity can't be reached
    if (v == 0)
    {
      if (t < T_s)
        return 0;
      else if (t < T_s + T / 2.0)
        return a * (t - T_s);
      else if (t <= T_s + T)
        return a * (T_s + T - t);
      else
        return 0;
    }
    // For max velocity can be reached
    else
    {
      if (t < T_s)
        return 0;
      else if (t < T_s + v / a)
        return a * (t - T_s);
      else if (t < T_s + T - v / a)
        return v;
      else if (t <= T_s + T)
        return a * (T_s + T - t);
      else
        return 0;
    }
  }
  scalar_t s(scalar_t t) const
  {
    // For max velocity can't be reached
    if (v == 0)
    {
      if (t < T_s)
        return 0;
      else if (t < T_s + T / 2.0)
        return a * (t - T_s) * (t - T_s) / 2.0;  // 1/2 * a * t^2
      else if (t <= T_s + T)
        return a * (T / 2.0) * (T / 2.0) - a * (T_s + T - t) * (T_s + T - t) / 2.0;
      else
        return 0;
    }
    // For max velocity can be reached
    else
    {
      if (t < T_s)
        return 0;
      else if (t < T_s + v / a)
        return a * (t - T_s) * (t - T_s) * 0.5;  // 1/2 * a * t^2
      else if (t < T_s + T - v / a)
        return v * (t - T_s) - v * v / (2.0 * a);
      else if (t <= T_s + T)
        return v * T - v * v / a - 0.5 * a * (T_s + T - t) * (T_s + T - t);
      else
        return 0;
    }
  }
};

struct PathPoint
{
  vector3_t p{ vector3_t::Zero() };         // position;
  scalar_t T_r{0.0};       // reachingTime;
  vector3_t dot_path{ vector3_t::Zero() };  // pathDerivative;  // d(theta)/ds = P_k - P_k-1
  TrapezoidalProfile s;                     // timeScalingFunction;
  PathPoint(vector3_t p, scalar_t T_r=0.0, vector3_t dot_path = vector3_t::Zero(),
            TrapezoidalProfile s = TrapezoidalProfile())
    : p(std::move(p)), T_r(T_r), dot_path(std::move(dot_path)), s(s)
  {
  }
};

class ArmTrajectoryPlanner
{
public:
  ArmTrajectoryPlanner() = default;
  ArmTrajectoryPlanner(scalar_t time);

  ~ArmTrajectoryPlanner() = default;

  int update(const vector3_t& new_pos,const scalar_t& v_m,const scalar_t& a_m, scalar_t time);
  vector_t getTwistConstraint(scalar_t time) const;
  vector3_t getLinearVelocityConstraint(scalar_t time) const;
  vector3_t getAngularVelocityConstraint(scalar_t time) const;

  scalar_t getPathPointsSize() const;

private:
  std::deque<PathPoint> path_;
};

}  // namespace legged_robot
}  // namespace ocs2
