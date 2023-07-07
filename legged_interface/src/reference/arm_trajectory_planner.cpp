#include "reference/arm_trajectory_planner.hpp"

namespace ocs2
{
namespace legged_robot
{

ArmTrajectoryPlanner::ArmTrajectoryPlanner(scalar_t time)
{
  // TODO modify first point
  path_.push_back(PathPoint(vector3_t{ 0.4, 0.0, 0.2 }, time));
}


int ArmTrajectoryPlanner::update(const vector3_t& new_pos, const scalar_t& v_m, const scalar_t& a_m, scalar_t time)
{
  auto first_point = path_.front();
  if (first_point.T_r < time && path_.size() == 1)
  {
    path_.front().T_r = time;
  }
  else if (path_.back().T_r < time && path_.size() > 1)
  {
    path_.pop_front();
  }
  auto last_point = path_.back();
  // if new point is near to last point, return
  if ((new_pos - last_point.p).norm() < 0.01)
  {
    return 1;
  }
  else
  {
    auto dot_path = new_pos - last_point.p;
    scalar_t path_length = dot_path.norm();
    if (path_length > v_m * v_m / a_m)
    {  // means v_m will be reached
      scalar_t t_a = v_m / a_m;
      scalar_t t_b = (dot_path.norm() - v_m * v_m / a_m) / v_m;
      scalar_t T = 2 * t_a + t_b;
      scalar_t v = a_m / (T * a_m - v_m);
      scalar_t a = a_m * a_m / (v_m * (T * a_m - v_m));
      scalar_t T_r = last_point.T_r + T;
      TrapezoidalProfile s(v, a, T, last_point.T_r);
      PathPoint new_point(new_pos, T_r, dot_path, s);
      path_.push_back(new_point);
      return 0;
    }
    else
    {  // means v_m won't be reached
      scalar_t T = 2 * sqrt(path_length / a_m);
      scalar_t v = 0.0;
      scalar_t a = 4 / T / T;
      scalar_t T_r = last_point.T_r + T;
      TrapezoidalProfile s(v, a, T, last_point.T_r);
      PathPoint new_point(new_pos, T_r, dot_path, s);
      path_.push_back(new_point);
      return 0;
    }
  }
}

vector_t ArmTrajectoryPlanner::getTwistConstraint(scalar_t time) const
{
  return vector_t::Zero(6);
}

vector3_t ArmTrajectoryPlanner::getLinearVelocityConstraint(scalar_t time) const
{
  assert(!path_.empty());
  //fing the first point that is after time
  auto it = path_.begin();
  while (it->T_r <= time && it != path_.end())
  {
    it++;
  }  
  if (it == path_.end())
  {
    return vector3_t::Zero();
  }
  else
  {
    return it->s.dots(time)*it->dot_path;
  }

}

vector3_t ArmTrajectoryPlanner::getLinearPositionConstraint(scalar_t time) const{
  assert(!path_.empty());
  auto it = path_.begin();
  while (it->T_r <= time && it != path_.end())
  {
    it++;
  }  
  if (it == path_.end())
  {
    return path_.back().p;
  }
  else
  {
    return (it->s.s(time)-1)*it->dot_path+it->p;
  }
}

scalar_t ArmTrajectoryPlanner::getPathPointsSize() const{
  return path_.size();
}
}  // namespace legged_robot
}  // namespace ocs2