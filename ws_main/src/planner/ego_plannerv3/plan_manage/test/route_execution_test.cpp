#include <gtest/gtest.h>
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
class RouteExecutionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fsm_.reset(new EGOReplanFSM);
    fsm_->have_route_ = true;
    fsm_->have_odom_ = true;
    fsm_->route_replan_use_guide_path_ = true;
    fsm_->route_local_window_length_ = 100.0;
    fsm_->odom_vel_.setZero();
    fsm_->odom_acc_.setZero();
    fsm_->odom_omega_.setZero();
    fsm_->ego_state_trigger_vel_thresh_ = 0.15;
    fsm_->ego_state_trigger_acc_thresh_ = 0.30;
    fsm_->ego_state_trigger_yaw_rate_thresh_ = 0.20;
    fsm_->ego_state_trigger_hold_time_ = 0.20;
  }

  void setRoute(const std::vector<Eigen::Vector3d> &points)
  {
    fsm_->route_wps_ = points;
    fsm_->route_yaws_.assign(points.size(), 0.0);
    fsm_->route_yaws_unwrapped_.assign(points.size(), 0.0);
    fsm_->odom_pos_ = points.front();
    fsm_->final_goal_ = points.back();
    fsm_->route_requested_goal_ = points.back();
    fsm_->rebuildRouteGeometry();
  }

  std::vector<Eigen::Vector3d> guide()
  {
    return fsm_->buildRouteGuidePath(fsm_->odom_pos_);
  }

  void checkShortEndpoint()
  {
    const auto path = guide();
    ASSERT_EQ(path.size(), 3u);
    EXPECT_NEAR(path.back().x(), 1.03, 1e-12);
  }

  void checkCornerBarrier()
  {
    ASSERT_EQ(fsm_->route_corners_.size(), 2u);
    EXPECT_TRUE(guide().back().isApprox(Eigen::Vector3d(4, 0, 1)));
    size_t index = 0;
    Eigen::Vector3d projection;
    double t = 0.0, s = 0.0;
    ASSERT_TRUE(fsm_->projectToRoute(Eigen::Vector3d(3.9, 0.3, 1), 0,
                                     index, projection, t, s));
    EXPECT_EQ(index, 0u); // The unvisited corner cannot be skipped.
    EXPECT_DOUBLE_EQ(projection.y(), 0.0);
  }

  void checkNoEarlyClosedCompletion()
  {
    EXPECT_TRUE(fsm_->routeReachedFinal()); // Position alone is insufficient.
    EXPECT_FALSE(fsm_->tryFinishRouteByOdom());
    EXPECT_TRUE(fsm_->route_finish_stable_since_.isZero());
  }

  void checkMovingVehicleNotFinished()
  {
    fsm_->odom_pos_ = fsm_->route_wps_.back();
    fsm_->odom_vel_ = Eigen::Vector3d(0.4, 0, 0);
    EXPECT_FALSE(fsm_->tryFinishRouteByOdom());
    EXPECT_TRUE(fsm_->route_finish_stable_since_.isZero());
    fsm_->odom_vel_.setZero();
    EXPECT_FALSE(fsm_->tryFinishRouteByOdom()); // Still requires the hold time.
    EXPECT_FALSE(fsm_->route_finish_stable_since_.isZero());
  }

  std::unique_ptr<EGOReplanFSM> fsm_;
};

TEST_F(RouteExecutionTest, PreservesShortFinalSegment)
{
  setRoute({{0, 0, 1}, {1, 0, 1}, {1.03, 0, 1}});
  checkShortEndpoint();
}

TEST_F(RouteExecutionTest, StopsAtTriangleCorners)
{
  setRoute({{0, 0, 1}, {4, 0, 1}, {4, 4, 1}, {0, 0, 1}});
  checkCornerBarrier();
}

TEST_F(RouteExecutionTest, ClosedRouteCannotFinishAtItsStart)
{
  setRoute({{0, 0, 1}, {4, 0, 1}, {4, 4, 1}, {0, 0, 1}});
  checkNoEarlyClosedCompletion();
}

TEST_F(RouteExecutionTest, RequiresStoppedOdometryAndStableHold)
{
  setRoute({{0, 0, 1}, {1, 0, 1}});
  checkMovingVehicleNotFinished();
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "route_execution_test", ros::init_options::AnonymousName);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}