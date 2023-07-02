#include "reference/arm_trajectory_planner.hpp"
#include "gtest/gtest.h"

namespace ocs2
{
namespace legged_robot
{

class TrapezoidalProfileTest : public testing::Test
{
protected:
  TrapezoidalProfileTest()
  {
    tp_ = TrapezoidalProfile{ 1.0, 1.0, 4.0, 0.0 };
    tp2_ = TrapezoidalProfile{ 0.0, 1.0, 4.0, 0.0 };
    tp3_ = TrapezoidalProfile{ 1.0, 1.0, 4.0, 1.3 };
    tp4_ = TrapezoidalProfile{ 0.0, 1.0, 4.0, 2.3 };

    auto new_pos1 = vector3_t{ 10.0, 0.0, 0.0 };
    atp_.update(new_pos1, 1, 1, 0);
    auto new_pos2 = vector3_t{ 6.0, 8.0, 0.0 };
    atp2_.update(new_pos2, 1, 1, 0);
    auto new_pos3 = vector3_t{ 6.0, 8.0+6.0, 8.0 };
    atp2_.update(new_pos3, 1, 1, 11.5);

  }

  TrapezoidalProfile tp_;
  TrapezoidalProfile tp2_;
  TrapezoidalProfile tp3_;
  TrapezoidalProfile tp4_;

  ArmTrajectoryPlanner atp_{0.0};
  ArmTrajectoryPlanner atp2_{0.0};
};

TEST_F(TrapezoidalProfileTest, ddots_test)
{
  EXPECT_EQ(tp_.ddots(0.0), 1.0);
  EXPECT_EQ(tp_.ddots(0.5), 1.0);
  EXPECT_EQ(tp_.ddots(1.0), 0.0);
  EXPECT_EQ(tp_.ddots(2.0), 0.0);
  EXPECT_EQ(tp_.ddots(2.5), 0.0);
  EXPECT_EQ(tp_.ddots(3.0), -1.0);
  EXPECT_EQ(tp_.ddots(3.5), -1.0);

  EXPECT_EQ(tp2_.ddots(0.0), 1.0);
  EXPECT_EQ(tp2_.ddots(1.0), 1.0);
  EXPECT_EQ(tp2_.ddots(2.0), -1.0);
  EXPECT_EQ(tp2_.ddots(3.0), -1.0);
  EXPECT_EQ(tp2_.ddots(4.0), -1.0);

  EXPECT_EQ(tp3_.ddots(0.0 + 1.3), 1.0);
  EXPECT_EQ(tp3_.ddots(0.5 + 1.3), 1.0);
  EXPECT_EQ(tp3_.ddots(1.0 + 1.3), 0.0);
  EXPECT_EQ(tp3_.ddots(2.0 + 1.3), 0.0);
  EXPECT_EQ(tp3_.ddots(2.5 + 1.3), 0.0);
  EXPECT_EQ(tp3_.ddots(3.0 + 1.3), -1.0);
  EXPECT_EQ(tp3_.ddots(3.5 + 1.3), -1.0);

  EXPECT_EQ(tp4_.ddots(0.0 + 2.3), 1.0);
  EXPECT_EQ(tp4_.ddots(1.0 + 2.3), 1.0);
  EXPECT_EQ(tp4_.ddots(2.0 + 2.3), -1.0);
  EXPECT_EQ(tp4_.ddots(3.0 + 2.3), -1.0);
  EXPECT_EQ(tp4_.ddots(4.0 + 2.3), -1.0);
}

TEST_F(TrapezoidalProfileTest, dots_test)
{
  EXPECT_EQ(tp_.dots(0.0), 0.0);
  EXPECT_EQ(tp_.dots(0.5), 0.5);
  EXPECT_EQ(tp_.dots(1.0), 1.0);
  EXPECT_EQ(tp_.dots(1.5), 1.0);
  EXPECT_EQ(tp_.dots(2.0), 1.0);
  EXPECT_EQ(tp_.dots(3.0), 1.0);
  EXPECT_EQ(tp_.dots(3.5), 0.5);
  EXPECT_EQ(tp_.dots(4.0), 0.0);

  EXPECT_EQ(tp2_.dots(0.0), 0.0);
  EXPECT_EQ(tp2_.dots(1.0), 1.0);
  EXPECT_EQ(tp2_.dots(2.0), 2.0);
  EXPECT_EQ(tp2_.dots(3.0), 1.0);
  EXPECT_EQ(tp2_.dots(4.0), 0.0);

  EXPECT_EQ(tp3_.dots(0.0 + 1.3), 0.0);
  EXPECT_EQ(tp3_.dots(0.5 + 1.3), 0.5);
  EXPECT_EQ(tp3_.dots(1.0 + 1.3), 1.0);
  EXPECT_EQ(tp3_.dots(1.5 + 1.3), 1.0);
  EXPECT_EQ(tp3_.dots(2.0 + 1.3), 1.0);
  EXPECT_EQ(tp3_.dots(3.0 + 1.3), 1.0);
  EXPECT_EQ(tp3_.dots(3.5 + 1.3), 0.5);
  EXPECT_EQ(tp3_.dots(4.0 + 1.3), 0.0);

  EXPECT_EQ(tp4_.dots(0.0 + 2.3), 0.0);
  EXPECT_EQ(tp4_.dots(1.0 + 2.3), 1.0);
  EXPECT_EQ(tp4_.dots(2.0 + 2.3), 2.0);
  EXPECT_EQ(tp4_.dots(3.0 + 2.3), 1.0);
  EXPECT_EQ(tp4_.dots(4.0 + 2.3), 0.0);
}

TEST_F(TrapezoidalProfileTest, s_test)
{
  EXPECT_EQ(tp_.s(0.0), 0.0);
  EXPECT_EQ(tp_.s(0.5), 0.125);
  EXPECT_EQ(tp_.s(1.0), 0.5);
  EXPECT_EQ(tp_.s(1.5), 1.0);
  EXPECT_EQ(tp_.s(2.0), 1.5);
  EXPECT_EQ(tp_.s(3.0), 2.5);
  EXPECT_EQ(tp_.s(3.5), 2.875);
  EXPECT_EQ(tp_.s(4.0), 3.0);

  EXPECT_EQ(tp2_.s(0.0), 0.0);
  EXPECT_EQ(tp2_.s(1.0), 0.5);
  EXPECT_EQ(tp2_.s(2.0), 2.0);
  EXPECT_EQ(tp2_.s(3.0), 3.5);
  EXPECT_EQ(tp2_.s(4.0), 4.0);

  EXPECT_EQ(tp3_.s(0.0 + 1.3), 0.0);
  EXPECT_EQ(tp3_.s(0.5 + 1.3), 0.125);
  EXPECT_NEAR(tp3_.s(1.0 + 1.3), 0.5, 1e-6);
  EXPECT_NEAR(tp3_.s(1.5 + 1.3), 1.0, 1e-6);
  EXPECT_NEAR(tp3_.s(2.0 + 1.3), 1.5, 1e-6);
  EXPECT_EQ(tp3_.s(3.0 + 1.3), 2.5);
  EXPECT_EQ(tp3_.s(3.5 + 1.3), 2.875);
  EXPECT_EQ(tp3_.s(4.0 + 1.3), 3.0);

  EXPECT_EQ(tp4_.s(0.0 + 2.3), 0.0);
  EXPECT_EQ(tp4_.s(1.0 + 2.3), 0.5);
  EXPECT_EQ(tp4_.s(2.0 + 2.3), 2.0);
  EXPECT_EQ(tp4_.s(3.0 + 2.3), 3.5);
  EXPECT_EQ(tp4_.s(4.0 + 2.3), 4.0);
}

TEST_F(TrapezoidalProfileTest, linear_vel_test){
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(0.0)(0), 0.0, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(0.5)(0), 0.5, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(1.0)(0), 1.0, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(1.5)(0), 1.0, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(9.0)(0), 1.0, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(10.0)(0), 1.0, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(10.5)(0), 0.5, 1e-6);
  EXPECT_NEAR(atp_.getLinearVelocityConstraint(11.0)(0), 0.0, 1e-6);

}

TEST_F(TrapezoidalProfileTest, linear_vel_test2){
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(0.0)(0), 0.0, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(0.5)(0), 0.5*0.6, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(1.0)(0), 1.0*0.6, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(1.5)(0), 1.0*0.6, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(9.0)(0), 1.0*0.6, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(10.0)(0), 1.0*0.6, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(10.5)(0), 0.5*0.6, 1e-6);
  // EXPECT_NEAR(atp2_.getLinearVelocityConstraint(11.0)(0), 0.0, 1e-6);
  EXPECT_EQ(atp2_.getPathPointsSize(), 2);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(11.5)(0), 0.0, 1e-6);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(15.0)(0), 0.0, 1e-6);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(11.5)(1), 0.5*0.6, 1e-6);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(12.5)(1), 1.0*0.6, 1e-6);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(14.5)(1), 1.0*0.6, 1e-6);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(21.5)(1), 0.5*0.6, 1e-6);
  EXPECT_NEAR(atp2_.getLinearVelocityConstraint(22.0)(1), 0.0, 1e-6);
}

}  // namespace legged_robot
}  // namespace ocs2
