#include <gtest/gtest.h>
#include <plan_manage/ego_replan_fsm.h>
#include <quadrotor_msgs/ScanGoalFeedback.h>

namespace ego_planner
{
class ScanGoalFeedbackTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fsm_.reset(new EGOReplanFSM);
    fsm_->scan_active_ = true;
    fsm_->scan_terminal_sent_ = false;
    fsm_->scan_session_id_ = "sess-1";
    fsm_->scan_request_id_ = 7;
    fsm_->scan_waypoint_id_ = 3;
    fsm_->scan_execution_goal_ = Eigen::Vector3d(1, 0, 1);
    fsm_->scan_requested_goal_ = Eigen::Vector3d(1, 0, 1);
    fsm_->scan_reach_tolerance_ = 1.0;
    fsm_->scan_odom_max_age_ = 1.0;
    fsm_->odom_pos_ = Eigen::Vector3d(1, 0, 1);
    fsm_->last_odom_stamp_ = ros::Time::now();
    fsm_->scan_event_seq_ = 0;
  }

  // 所有私有成员访问都封装在 fixture 本体内 (friend 只对 fixture 生效)。
  void checkReachedWhenNearAndFresh()
  {
    fsm_->scanOnTrajectoryEnded(true);
    EXPECT_TRUE(fsm_->scan_terminal_sent_);
    EXPECT_EQ(fsm_->scan_last_terminal_status_, quadrotor_msgs::ScanGoalFeedback::REACHED);
    EXPECT_FALSE(fsm_->scan_active_);
  }

  void checkReachedWhenNearOffset()
  {
    // touch_goal=false (目标被改写到附近安全点), 但 odom 实际已在执行点容差内 -> 仍判 REACHED。
    fsm_->scanOnTrajectoryEnded(false);
    EXPECT_TRUE(fsm_->scan_terminal_sent_);
    EXPECT_EQ(fsm_->scan_last_terminal_status_, quadrotor_msgs::ScanGoalFeedback::REACHED);
    EXPECT_FALSE(fsm_->scan_active_);
  }

  void checkFailedWhenUnreachable()
  {
    fsm_->odom_pos_ = Eigen::Vector3d(5, 0, 1); // 远离执行点
    fsm_->scanOnTrajectoryEnded(false);
    EXPECT_TRUE(fsm_->scan_terminal_sent_);
    EXPECT_EQ(fsm_->scan_last_terminal_status_, quadrotor_msgs::ScanGoalFeedback::FAILED);
    EXPECT_FALSE(fsm_->scan_active_);
  }

  void checkFailedWhenReachedButOffset()
  {
    fsm_->odom_pos_ = Eigen::Vector3d(5, 0, 1);
    fsm_->scanOnTrajectoryEnded(true);
    EXPECT_TRUE(fsm_->scan_terminal_sent_);
    EXPECT_EQ(fsm_->scan_last_terminal_status_, quadrotor_msgs::ScanGoalFeedback::FAILED);
  }

  void checkFailedWhenOdomStale()
  {
    fsm_->last_odom_stamp_ = ros::Time(0);
    fsm_->scanOnTrajectoryEnded(true);
    EXPECT_EQ(fsm_->scan_last_terminal_status_, quadrotor_msgs::ScanGoalFeedback::FAILED);
  }

  void checkSupersedeSendsCancelledAndClears()
  {
    fsm_->scanSupersede("superseded_by_aim");
    EXPECT_TRUE(fsm_->scan_terminal_sent_);
    EXPECT_EQ(fsm_->scan_last_terminal_status_, quadrotor_msgs::ScanGoalFeedback::CANCELLED);
    EXPECT_FALSE(fsm_->scan_active_);
  }

  void checkClearKeepsDedupRecord()
  {
    fsm_->scanOnTrajectoryEnded(true);
    fsm_->scanClearContext();
    EXPECT_FALSE(fsm_->scan_active_);
    EXPECT_EQ(fsm_->scan_session_id_, std::string("sess-1"));
    EXPECT_EQ(fsm_->scan_request_id_, 7u);
    EXPECT_TRUE(fsm_->scan_terminal_sent_);
  }

  void checkSupersedeIdempotentWhenInactive()
  {
    fsm_->scan_active_ = false;
    fsm_->scan_terminal_sent_ = false;
    fsm_->scanSupersede("noop");
    EXPECT_FALSE(fsm_->scan_terminal_sent_);
  }

  std::unique_ptr<EGOReplanFSM> fsm_;
};

TEST_F(ScanGoalFeedbackTest, ReachedWhenNearAndFresh) { checkReachedWhenNearAndFresh(); }
TEST_F(ScanGoalFeedbackTest, ReachedWhenNearOffset) { checkReachedWhenNearOffset(); }
TEST_F(ScanGoalFeedbackTest, FailedWhenUnreachable) { checkFailedWhenUnreachable(); }
TEST_F(ScanGoalFeedbackTest, FailedWhenReachedButOffset) { checkFailedWhenReachedButOffset(); }
TEST_F(ScanGoalFeedbackTest, FailedWhenOdomStale) { checkFailedWhenOdomStale(); }
TEST_F(ScanGoalFeedbackTest, SupersedeSendsCancelledAndClears) { checkSupersedeSendsCancelledAndClears(); }
TEST_F(ScanGoalFeedbackTest, ClearKeepsDedupRecord) { checkClearKeepsDedupRecord(); }
TEST_F(ScanGoalFeedbackTest, SupersedeIdempotentWhenInactive) { checkSupersedeIdempotentWhenInactive(); }
} // namespace ego_planner

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_goal_feedback_test", ros::init_options::AnonymousName);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
