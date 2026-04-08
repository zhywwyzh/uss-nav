#ifndef _GRID_MAP_
#define _GRID_MAP_

#include <thread>
#include <mutex>
#include <chrono>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>
#include <plan_env/visualization.hpp>

// Note: origin(0.0, 0.0, 0.0) is mapped to the cornor of grid(0, 0, 0)

#define INFO_MSG(str) do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str) do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str) do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str) do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

#define logit(x) (log((x) / (1 - (x))))
#define occ_high(id) (md_.occ_buf_[id] >= mp_.min_occupancy_log_)                   // consistancy purpose
#define occ_high2(id, obj) (obj->md_.occ_buf_[id] >= obj->mp_.min_occupancy_log_)   // consistancy purpose
#define occ_low(id) (md_.occ_buf_[id] <= mp_.max_clear_log_)                        // consistancy purpose
#define occ_low2(id, obj) (obj->md_.occ_buf_[id] <= obj->mp_.max_clear_log_)        // consistancy purpose
#define isocc(id) (md_.occ_buf_inf_[id] >= GRID_MAP_OBS_FLAG)                       // consistancy purpose
#define isocc2(id, obj) (obj->md_.occ_buf_inf_[id] >= GRID_MAP_OBS_FLAG)            // consistancy purpose
#define isocc_out(id) (md_.occ_buf_inf_output_[id] >= GRID_MAP_OBS_FLAG)            // consistancy purpose
#define isocc_out2(id, obj) (obj->md_.occ_buf_inf_output_[id] >= GRID_MAP_OBS_FLAG) // consistancy purpose
#define GRID_MAP_OBS_FLAG 10000                                                     // MUST > 0!!! to distinguish inflated girds and real-obs grids.
#define GRID_MAP_UNKNOWN_FLAG -15000                                                // MUST < 0!!!
#define GRID_MAP_UNKNOWN_ESDF_FLAG 15000.0f                                         // MUST BE A BIG VALUE (ESDF REQUIRED)
#define GRID_MAP_UNKNOWN_ESDF_FLAG_COMP 14999.0f                                    // GRID_MAP_UNKNOWN_ESDF_FLAG - 1.0f (ESDF REQUIRED)
#define GRID_MAP_OUTOFREGION_FLAG 20000                                             // MUST BE A BIG VALUE (ESDF REQUIRED)
#define GRID_MAP_NEW_PLATFORM_TEST false
#define COMPUTE_NEGTIVE_EDT false

#define likely(x) __builtin_expect((x), 1)
#define unlikely(x) __builtin_expect((x), 0)

using namespace std;

// constant parameters

struct MappingParameters
{
  bool have_initialized_ = false;

  /* map properties */
  Eigen::Vector3d local_update_range3d_;
  Eigen::Vector3i local_update_range3i_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  int inf_grid_;
  string frame_id_;
  int pose_type_;
  bool enable_virtual_walll_;
  double virtual_ceil_, virtual_ground_;
  bool esdf_enable_{true};

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;
  int clear_target_pos_num_;

  /* depth image projection filtering */
  double k_depth_scaling_factor_;
  int skip_pixel_;
  /* depth image filter parameter */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                                          // occupancy probability
  float prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_, max_clear_log_; // logit of occupancy probability
  double min_ray_length_;                                                                                  // range of doing raycasting
  double fading_time_;
  double unknown_flag_;
  int hit_accum_max_;
  int hit_accum_threshold_;

  /* visualization and computation time display */
  bool show_occ_time_;
  double visual_inflat_map_height_;

  double light_height_; //trick, filter light
  double light_length_; //trick, filter light
  int brightness_threshold_; //trick, allowed maximum brightness

  /* multi-thread */
  // bool multi_thread_{false}; // the planner don't need precise sync with the larger map
  std::string name_{std::string("Map")};
};

// intermediate mapping data for fusion

struct MappingData
{
  Eigen::Vector3i center_last3i_;
  Eigen::Vector3i update_range_lb3i_, update_range_ub3i_;
  // rb: ring buffer
  Eigen::Vector3i rb_orig3i_;
  Eigen::Vector3d rb_lb3d_;
  Eigen::Vector3i rb_lb3i_;
  Eigen::Vector3d rb_ub3d_;
  Eigen::Vector3i rb_ub3i_;
  Eigen::Vector3i rb_size3i_;
  int xy_step_;
  Eigen::Vector3i rb_inf_orig3i_;
  Eigen::Vector3d rb_inf_lb3d_;
  Eigen::Vector3i rb_inf_lb3i_;
  Eigen::Vector3d rb_inf_ub3d_;
  Eigen::Vector3i rb_inf_ub3i_;
  Eigen::Vector3i rb_inf_size3i_;
  int xy_step_inf_;

  // main map data, occupancy of each voxel
  std::vector<float> occ_buf_, occ_buf_output_;
  std::vector<int16_t> occ_buf_inf_, occ_buf_inf_output_;
  std::vector<double> dist_buf_;
  std::vector<double> tmp_buf0_, tmp_buf1_, tmp_buf2_, tmp_buf3_;
  Eigen::Vector3i tmpbuf_offset_;
#if COMPUTE_NEGTIVE_EDT
  std::vector<double> dist_buf_neg_, dist_buf_all_;
  std::vector<int8_t> occ_buf_neg_;
#endif

  // detect target position
  Eigen::Vector3d target_pos_;
  bool target_pos_update_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_;
  Eigen::Matrix3d camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data
  cv::Mat depth_image_, gray_image_;

  // flags of map state
  bool has_first_depth_;
  bool has_odom_;
  bool finish_occupancy_;

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_have_ever_received_depth_;
  bool flag_have_ever_received_pc_;

  // depth image projected point cloud
  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt_;

  // flag buffers for speeding up raycasting
  vector<short> count_hit_, count_hit_and_miss_;
  vector<short> hit_accum_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;

  vector<Eigen::Vector3i> cache_voxel_;
  int cache_voxel_cnt_;

  int map_voxel_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TimeStatistics
{
  int updatetimes = 0;
  double raycasttime = 0;
  double max_raycasttime = 0;
  double inflationtime = 0;
  double max_inflationtime = 0;
  double esdftime = 0;
  double max_esdftime = 0;
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap() {}

  enum OCCUPANCY { FREE, OCCUPIED, UNKNOWN };

  void initMap(ros::NodeHandle &nh);
  void paramAdjust(double reso, double rangex, double rangey, double rangez, bool esdf_en, bool inf_en, bool multi_thread, std::string suffix);
  inline int getRawOccupancy(const Eigen::Vector3d &pos) const;
  inline int getRawOccupancy(const Eigen::Vector3i &idx) const;
  inline int getRawOccupancyOutput(const Eigen::Vector3d &pos) const;
  inline int getRawOccupancyOutput(const Eigen::Vector3i &idx) const;
  inline int getOccupancy(const Eigen::Vector3d &pos, bool ignore_virtual_wall = false) const;
  inline int getInflateOccupancy(const Eigen::Vector3d &pos) const;
  inline int getRawInflateOccupancy(const Eigen::Vector3d &pos) const;
  inline double getDistance(const Eigen::Vector3d &pos) const;
  inline double getDistance(const Eigen::Vector3i &id) const; // not recommaneded
  inline double getDistancePessi(const Eigen::Vector3d &pos) const;
  inline double getResolution() const;
  inline void setTarget(const Eigen::Vector3d &target_pos, const bool &target_pos_valid);
  bool getOdomDepthTimeout() const { return md_.flag_have_ever_received_depth_ ? (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ : false; }
  inline bool evaluateESDFWithGrad(const Eigen::Vector3d &pos, double &dist, double *grad) const;
  inline std::pair<double, double> getFOV() const { return std::pair<double, double>(mp_.cx_ / mp_.fx_, mp_.cy_ / mp_.fy_); };
  inline Eigen::Vector3d getGridCenter(Eigen::Vector3d pos) const { return globalIdx2Pos(pos2GlobalIdx(pos)); };
  inline Eigen::Vector3d getLocalMapRange() const { return mp_.local_update_range3d_; };
  void LockCopyToOutputInfMap(bool lock);
  void LockCopyToOutputAllMap(bool lock);

  int getu(int cols);
  inline int getVoxelNum() const { return md_.map_voxel_num_; };
  void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
  void getUpdatedBoxIdx(Eigen::Vector3i& bmin_inx, Eigen::Vector3i& bmax_inx);
  inline bool isInited() const {return (md_.has_odom_ && (md_.flag_have_ever_received_depth_ || md_.flag_have_ever_received_pc_)); }

  inline bool isInBuf(const Eigen::Vector3d &pos) const;
  inline bool isInBuf(const Eigen::Vector3i &idx) const;
  inline Eigen::Vector3d globalIdx2Pos(const Eigen::Vector3i &id) const;  // 1.69ns
  inline Eigen::Vector3i pos2GlobalIdx(const Eigen::Vector3d &pos) const; // 0.13ns
  inline size_t globalIdx2BufIdx(const Eigen::Vector3i &id) const;        // 2.2ns
  inline Eigen::Vector3i globalIdx2TmpBuf3dIdx(const Eigen::Vector3i &id) const;

  typedef std::shared_ptr<GridMap> Ptr;

  std::shared_ptr<visualization::Visualization> visPtr_;


  std::mutex mtx_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  TimeStatistics ts_;
  std::mutex mtx_vis_, mtx_memcpy_inf_, mtx_memcpy_;

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    // INVALID_IDX = -10000
  };


  inline size_t TmpBuf3dIdx2TmpBuf1dIdx(const Eigen::Vector3i &id) const;
  inline size_t globalIdx2InfBufIdx(const Eigen::Vector3i &id) const; // 2.2ns
  inline Eigen::Vector3i BufIdx2GlobalIdx(size_t address) const;      // 10.18ns
  inline Eigen::Vector3i infBufIdx2GlobalIdx(size_t address) const;   // 10.18ns
  inline bool isInInfBuf(const Eigen::Vector3d &pos) const;
  inline bool isInInfBuf(const Eigen::Vector3i &idx) const;
  inline Eigen::Vector3i min3i(Eigen::Vector3i in1, Eigen::Vector3i in2);
  inline Eigen::Vector3i max3i(Eigen::Vector3i in1, Eigen::Vector3i in2);

  void publishMap() const;
  void publishMapInflate() const;
  void publishESDF() const;
  void publishUpdateRange(); 


  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom, const sensor_msgs::ImageConstPtr &gray);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom);

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  void updateESDF3d();

  static void visCallback(void *obj);
  static void fadingCallback(void *obj);

  void clearBuffer(char casein, int bound);

  // main update process
  static void updateOccupancy(void *obj);
  static void updateOccupancyPC(void *obj, const sensor_msgs::PointCloud2ConstPtr &img);
  static void processCloud(void *obj, const sensor_msgs::PointCloud2ConstPtr &img);
  void projectPC(pcl::PointCloud<pcl::PointXYZ> &pc);
  void moveRingBuffer();
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();
  void CopyToOutputMap();

  inline void changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i &global_idx);
  inline int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  void initMapBoundary();

  // debug
  void testIndexingCost();
  void cheakMapConsistancy(std::string where);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, sensor_msgs::Image>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> gray_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_, extrinsic_sub_;
  ros::Publisher map_pub_, map_inf_pub_, map_unknown_pub_, esdf_pub_, update_range_pub_;
  ros::Timer fading_timer_;
  ros::Publisher filterlight_pub_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline functions
 * ============================== */

inline int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  // if (occ != 1 && occ != 0)
  //   return INVALID_IDX;

  Eigen::Vector3i id = pos2GlobalIdx(pos);
  int idx_ctns = globalIdx2BufIdx(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

inline void GridMap::changeInfBuf(const bool dir, const int inf_buf_idx, const Eigen::Vector3i &global_idx)
{
  int inf_grid = mp_.inf_grid_;
  if (dir)
  {
    if (md_.occ_buf_inf_[inf_buf_idx] == GRID_MAP_UNKNOWN_FLAG)
      md_.occ_buf_inf_[inf_buf_idx] = 0;
    md_.occ_buf_inf_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
  }
  else
    md_.occ_buf_inf_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;

  for (int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    for (int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
      for (int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
      {
        Eigen::Vector3i id_inf(global_idx + Eigen::Vector3i(x_inf, y_inf, z_inf));
#if GRID_MAP_NEW_PLATFORM_TEST
        if (isInInfBuf(id_inf))
        {
          size_t id_inf_buf = globalIdx2InfBufIdx(id_inf);
          if (dir)
          {
            if (md_.occ_buf_inf_[id_inf_buf] == GRID_MAP_UNKNOWN_FLAG)
              md_.occ_buf_inf_[id_inf_buf] = 0;
            ++md_.occ_buf_inf_[id_inf_buf];
          }
          else
          {
            --md_.occ_buf_inf_[id_inf_buf];
            if (md_.occ_buf_inf_[id_inf_buf] < 0) // An error case
            {
              ROS_ERROR("A negtive value of nearby obstacle number! reset the map.");
              fill(md_.occ_buf_.begin(), md_.occ_buf_.end(), mp_.clamp_min_log_);
              fill(md_.occ_buf_inf_.begin(), md_.occ_buf_inf_.end(), GRID_MAP_UNKNOWN_FLAG);
              fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
              fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
              fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
              fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
              fill(md_.cache_voxel_.begin(), md_.cache_voxel_.end(), Eigen::Vector3i(0, 0, 0));
            }
          }

          if (md_.occ_buf_inf_[id_inf_buf] < 0)
          {
            cout << "2 occ=" << md_.occ_buf_inf_[id_inf_buf] << " id_inf_buf=" << id_inf_buf << " id_inf=" << id_inf.transpose() << " pos=" << globalIdx2Pos(id_inf).transpose() << endl;
          }
        }
        else
        {
          cout << "id_inf=" << id_inf.transpose() << " md_.rb_inf_ub3i_=" << md_.rb_inf_ub3i_.transpose() << " md_.rb_ub3i_=" << md_.rb_ub3i_.transpose() << endl;
          ROS_ERROR("isInInfBuf return false 1");
        }

#else
        size_t id_inf_buf = globalIdx2InfBufIdx(id_inf);
        if (dir)
        {
          if (md_.occ_buf_inf_[id_inf_buf] == GRID_MAP_UNKNOWN_FLAG)
            md_.occ_buf_inf_[id_inf_buf] = 0;
          ++md_.occ_buf_inf_[id_inf_buf];
        }
        else
        {
          --md_.occ_buf_inf_[id_inf_buf];
          if (unlikely(md_.occ_buf_inf_[id_inf_buf] < 0)) // An error case
          {
            ROS_ERROR("Negtive value (%d) of nearby obstacle number! reset the map.", md_.occ_buf_inf_[id_inf_buf]);
            fill(md_.occ_buf_.begin(), md_.occ_buf_.end(), mp_.clamp_min_log_);
            fill(md_.occ_buf_inf_.begin(), md_.occ_buf_inf_.end(), GRID_MAP_UNKNOWN_FLAG);
            fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
            fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
            fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
            fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
            fill(md_.cache_voxel_.begin(), md_.cache_voxel_.end(), Eigen::Vector3i(0, 0, 0));
          }
        }
#endif
      }
}

inline size_t GridMap::globalIdx2BufIdx(const Eigen::Vector3i &id) const
{
  int x_buffer = (id(0) - md_.rb_orig3i_(0)) % md_.rb_size3i_(0); // Do NOT remove the "%" operation! Otherwise will randomly crash
  int y_buffer = (id(1) - md_.rb_orig3i_(1)) % md_.rb_size3i_(1);
  int z_buffer = (id(2) - md_.rb_orig3i_(2)) % md_.rb_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.rb_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.rb_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.rb_size3i_(2);

  return md_.xy_step_ * z_buffer + md_.rb_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::globalIdx2TmpBuf3dIdx(const Eigen::Vector3i &id) const
{
  return id - md_.tmpbuf_offset_;
}

inline size_t GridMap::TmpBuf3dIdx2TmpBuf1dIdx(const Eigen::Vector3i &id) const
{
  return md_.xy_step_ * id(2) + md_.rb_size3i_(0) * id(1) + id(0);
}

inline size_t GridMap::globalIdx2InfBufIdx(const Eigen::Vector3i &id) const
{
  int x_buffer = (id(0) - md_.rb_inf_orig3i_(0)) % md_.rb_inf_size3i_(0); // Do NOT remove the "%" operation! Otherwise will randomly crash
  int y_buffer = (id(1) - md_.rb_inf_orig3i_(1)) % md_.rb_inf_size3i_(1);
  int z_buffer = (id(2) - md_.rb_inf_orig3i_(2)) % md_.rb_inf_size3i_(2);
  if (x_buffer < 0)
    x_buffer += md_.rb_inf_size3i_(0);
  if (y_buffer < 0)
    y_buffer += md_.rb_inf_size3i_(1);
  if (z_buffer < 0)
    z_buffer += md_.rb_inf_size3i_(2);

  return md_.xy_step_inf_ * z_buffer + md_.rb_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Eigen::Vector3i GridMap::BufIdx2GlobalIdx(size_t address) const
{
  int xy_grid_in_buffer = address % md_.xy_step_;
  int zid_in_buffer = address / md_.xy_step_;
  int yid_in_buffer = xy_grid_in_buffer / md_.rb_size3i_(0);
  int xid_in_buffer = xy_grid_in_buffer % md_.rb_size3i_(0);

  int xid_global = xid_in_buffer + md_.rb_orig3i_(0);
  if (xid_global > md_.rb_ub3i_(0))
    xid_global -= md_.rb_size3i_(0);
  int yid_global = yid_in_buffer + md_.rb_orig3i_(1);
  if (yid_global > md_.rb_ub3i_(1))
    yid_global -= md_.rb_size3i_(1);
  int zid_global = zid_in_buffer + md_.rb_orig3i_(2);
  if (zid_global > md_.rb_ub3i_(2))
    zid_global -= md_.rb_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

inline Eigen::Vector3i GridMap::infBufIdx2GlobalIdx(size_t address) const
{
  int xy_grid_in_buffer = address % md_.xy_step_inf_;
  int zid_in_buffer = address / md_.xy_step_inf_;
  int yid_in_buffer = xy_grid_in_buffer / md_.rb_inf_size3i_(0);
  int xid_in_buffer = xy_grid_in_buffer % md_.rb_inf_size3i_(0);

  int xid_global = xid_in_buffer + md_.rb_inf_orig3i_(0);
  if (xid_global > md_.rb_inf_ub3i_(0))
    xid_global -= md_.rb_inf_size3i_(0);
  int yid_global = yid_in_buffer + md_.rb_inf_orig3i_(1);
  if (yid_global > md_.rb_inf_ub3i_(1))
    yid_global -= md_.rb_inf_size3i_(1);
  int zid_global = zid_in_buffer + md_.rb_inf_orig3i_(2);
  if (zid_global > md_.rb_inf_ub3i_(2))
    zid_global -= md_.rb_inf_size3i_(2);

  return Eigen::Vector3i(xid_global, yid_global, zid_global);
}

// return: 
// out of map         -> (-1)
// GridMap::FREE      -> (0)
// GridMap::OCCUPIED  -> (1)
// GridMap::UNKNOWN   -> (2)
inline int GridMap::getRawOccupancy(const Eigen::Vector3d &pos) const
{
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  if (!isInBuf(pos))
    return -1;

  float occ = md_.occ_buf_[globalIdx2BufIdx(pos2GlobalIdx(pos))];

  // ROS_WARN_STREAM("pos: " << pos.transpose() << " = " << occ << " | clamp_min_log_: " << mp_.clamp_min_log_ << " | min_occupancy_log_: " << mp_.min_occupancy_log_);

  if (occ < mp_.clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_.min_occupancy_log_) return OCCUPIED;
  return FREE;
}

inline int GridMap::getRawOccupancy(const Eigen::Vector3i &idx) const
{
  Eigen::Vector3d pos = globalIdx2Pos(idx);
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  if (!isInBuf(idx))
    return -1;

  // ROS_INFO_STREAM("[getRawOccupancy] isInBuf(idx): " << isInBuf(idx));
  // ROS_INFO_STREAM("[getRawOccupancy] bufferidx(idx): " << globalIdx2BufIdx(idx) << " | " << getVoxelNum());

  float occ = md_.occ_buf_[globalIdx2BufIdx(idx)];
  if (occ < mp_.clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_.min_occupancy_log_) return OCCUPIED;
  return FREE;
}

// return: 
// out of map         -> (-1)
// GridMap::FREE      -> (0)
// GridMap::OCCUPIED  -> (1)
// GridMap::UNKNOWN   -> (2)
inline int GridMap::getRawOccupancyOutput(const Eigen::Vector3d &pos) const
{
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  if (!isInBuf(pos))
    return -1;

  float occ = md_.occ_buf_output_[globalIdx2BufIdx(pos2GlobalIdx(pos))];

  // ROS_WARN_STREAM("pos: " << pos.transpose() << " = " << occ << " | clamp_min_log_: " << mp_.clamp_min_log_ << " | min_occupancy_log_: " << mp_.min_occupancy_log_);

  if (occ < mp_.clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_.min_occupancy_log_) return OCCUPIED;
  return FREE;
}

inline int GridMap::getRawOccupancyOutput(const Eigen::Vector3i &idx) const
{
  Eigen::Vector3d pos = globalIdx2Pos(idx);
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  if (!isInBuf(idx))
    return -1;

  // ROS_INFO_STREAM("[getRawOccupancy] isInBuf(idx): " << isInBuf(idx));
  // ROS_INFO_STREAM("[getRawOccupancy] bufferidx(idx): " << globalIdx2BufIdx(idx) << " | " << getVoxelNum());

  float occ = md_.occ_buf_output_[globalIdx2BufIdx(idx)];
  if (occ < mp_.clamp_min_log_ - 1e-3) return UNKNOWN;
  if (occ > mp_.min_occupancy_log_) return OCCUPIED;
  return FREE;
}

// return:
// out of map         -> (-1)
// free               -> (0)
// occ                -> (1)
inline int GridMap::getOccupancy(const Eigen::Vector3d &pos, bool ignore_virtual_wall /* = false*/) const
{
  if (!ignore_virtual_wall && mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return -1;

  if (!isInBuf(pos))
    return 0;

  return isocc_out(globalIdx2InfBufIdx(pos2GlobalIdx(pos))) ? 1 : 0;
}

// return:
// GRID_MAP_OUTOFREGION_FLAG    -> (20000)
// free                         -> (0)
// occ                          -> (1)
inline int GridMap::getInflateOccupancy(const Eigen::Vector3d &pos) const
{
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return GRID_MAP_OUTOFREGION_FLAG;

  if (!isInInfBuf(pos))
    return 0;

  return int(md_.occ_buf_inf_output_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);

  // if (getRawOccupancyOutput(pos) == GridMap::UNKNOWN) return 1;
  // else return int(md_.occ_buf_inf_output_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

// return:
// GRID_MAP_OUTOFREGION_FLAG    -> (20000)
// GRID_MAP_UNKNOWN_FLAG        -> (-15000)
// free                         -> (0)
// occ                          -> (1)
inline int GridMap::getRawInflateOccupancy(const Eigen::Vector3d &pos) const
{
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    return GRID_MAP_OUTOFREGION_FLAG;

  if (!isInInfBuf(pos))
    return 0;

  return int(md_.occ_buf_inf_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]);
}

inline double GridMap::getDistance(const Eigen::Vector3d &pos) const
{
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_) || !isInInfBuf(pos))
    return (double)GRID_MAP_OUTOFREGION_FLAG;

#if COMPUTE_NEGTIVE_EDT
  return md_.dist_buf_all_[globalIdx2BufIdx(pos2GlobalIdx(pos))] - mp_.obstacles_inflation_;
#else
  return md_.dist_buf_[globalIdx2BufIdx(pos2GlobalIdx(pos))] - mp_.obstacles_inflation_;
#endif
}

inline double GridMap::getDistance(const Eigen::Vector3i &id) const // not recommaneded
{
  Eigen::Vector3d pos = globalIdx2Pos(id);
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_) || !isInInfBuf(id))
    return (double)GRID_MAP_OUTOFREGION_FLAG;

#if COMPUTE_NEGTIVE_EDT
  return md_.dist_buf_all_[globalIdx2BufIdx(pos2GlobalIdx(pos))];
#else
  return md_.dist_buf_[globalIdx2BufIdx(pos2GlobalIdx(pos))];
#endif
}

inline double GridMap::getDistancePessi(const Eigen::Vector3d &pos) const
{
  if (mp_.enable_virtual_walll_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_) || !isInInfBuf(pos))
    return 0.0f;

#if COMPUTE_NEGTIVE_EDT
  double dist = md_.dist_buf_all_[globalIdx2BufIdx(pos2GlobalIdx(pos))];
#else
  double dist = md_.dist_buf_[globalIdx2BufIdx(pos2GlobalIdx(pos))];
#endif

  if (dist > GRID_MAP_UNKNOWN_ESDF_FLAG_COMP)
    return 0.0f;
  return dist - mp_.obstacles_inflation_;
}

bool GridMap::evaluateESDFWithGrad(const Eigen::Vector3d &pos,
                                   double &dist,
                                   double *grad) const
{
  if (!isInBuf(pos))
    return false;

  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx = pos2GlobalIdx(pos_m);
  Eigen::Vector3d idx_pos = globalIdx2Pos(idx);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos = globalIdx2Pos(current_idx);
        sur_pts[x][y][z] = current_pos;
      }

  double dists[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++)
      {
        dists[x][y][z] = getDistance(sur_pts[x][y][z]);
        if (dists[x][y][z] > GRID_MAP_UNKNOWN_ESDF_FLAG_COMP)
          return false;
      }

  // trilinear interpolation
  double v00 = (1 - diff(0)) * dists[0][0][0] + diff(0) * dists[1][0][0];
  double v01 = (1 - diff(0)) * dists[0][0][1] + diff(0) * dists[1][0][1];
  double v10 = (1 - diff(0)) * dists[0][1][0] + diff(0) * dists[1][1][0];
  double v11 = (1 - diff(0)) * dists[0][1][1] + diff(0) * dists[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  dist = (1 - diff(2)) * v0 + diff(2) * v1;

  if (grad != NULL)
  {
    grad[2] = (v1 - v0) * mp_.resolution_inv_;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (dists[1][0][0] - dists[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (dists[1][1][0] - dists[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (dists[1][0][1] - dists[0][0][1]);
    grad[0] += diff[2] * diff[1] * (dists[1][1][1] - dists[0][1][1]);
    grad[0] *= mp_.resolution_inv_;
  }

  return true;
}

inline bool GridMap::isInBuf(const Eigen::Vector3d &pos) const
{
  if (pos(0) < md_.rb_lb3d_(0) || pos(1) < md_.rb_lb3d_(1) || pos(2) < md_.rb_lb3d_(2))
  {
    return false;
  }
  if (pos(0) >= md_.rb_ub3d_(0) || pos(1) >= md_.rb_ub3d_(1) || pos(2) >= md_.rb_ub3d_(2)) // MUST use ">=" !!!
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInBuf(const Eigen::Vector3i &idx) const
{
  if (idx(0) < md_.rb_lb3i_(0) || idx(1) < md_.rb_lb3i_(1) || idx(2) < md_.rb_lb3i_(2))
  {
    return false;
  }
  if (idx(0) > md_.rb_ub3i_(0) || idx(1) > md_.rb_ub3i_(1) || idx(2) > md_.rb_ub3i_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3d &pos) const
{
  if (pos(0) < md_.rb_inf_lb3d_(0) || pos(1) < md_.rb_inf_lb3d_(1) || pos(2) < md_.rb_inf_lb3d_(2))
  {
    return false;
  }
  if (pos(0) > md_.rb_inf_ub3d_(0) || pos(1) > md_.rb_inf_ub3d_(1) || pos(2) > md_.rb_inf_ub3d_(2))
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInInfBuf(const Eigen::Vector3i &idx) const
{
  if (idx(0) < md_.rb_inf_lb3i_(0) || idx(1) < md_.rb_inf_lb3i_(1) || idx(2) < md_.rb_inf_lb3i_(2))
  {
    return false;
  }
  if (idx(0) > md_.rb_inf_ub3i_(0) || idx(1) > md_.rb_inf_ub3i_(1) || idx(2) > md_.rb_inf_ub3i_(2))
  {
    return false;
  }
  return true;
}

inline Eigen::Vector3d GridMap::globalIdx2Pos(const Eigen::Vector3i &id) const // t ~ 0us
{
  return (id.cast<double>() + 0.5 * Eigen::Vector3d::Ones()) * mp_.resolution_;
  // return Eigen::Vector3d((id(0) + 0.5) * mp_.resolution_, (id(1) + 0.5) * mp_.resolution_, (id(2) + 0.5) * mp_.resolution_);
}

inline Eigen::Vector3i GridMap::pos2GlobalIdx(const Eigen::Vector3d &pos) const
{
  return (pos * mp_.resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
}

inline double GridMap::getResolution() const { return mp_.resolution_; }

inline void GridMap::setTarget(const Eigen::Vector3d &target_pos, const bool &target_pos_valid) { md_.target_pos_ = target_pos; md_.target_pos_update_ = target_pos_valid; }

inline Eigen::Vector3i GridMap::min3i(Eigen::Vector3i in1, Eigen::Vector3i in2)
{
  return Eigen::Vector3i(min(in1(0), in2(0)), min(in1(1), in2(1)), min(in1(2), in2(2)));
}

inline Eigen::Vector3i GridMap::max3i(Eigen::Vector3i in1, Eigen::Vector3i in2)
{
  return Eigen::Vector3i(max(in1(0), in2(0)), max(in1(1), in2(1)), max(in1(2), in2(2)));
}

class MapManager
{
public:
  MapManager() {}
  ~MapManager() {}

  GridMap::Ptr sml_, big_;
  GridMap::Ptr cur_;

  enum MAP_USE
  {
    SMALL,
    LARGE
  };

  void initMapManager(ros::NodeHandle &nh);
  void setMapUse(MAP_USE use);
  MAP_USE getMapUse() const { return map_use_; }
  inline int getOcc(const Eigen::Vector3d &pos) const;
  inline double getReso() const { return cur_->getResolution(); }

  typedef std::shared_ptr<MapManager> Ptr;

private:
  MAP_USE map_use_;
};

inline int MapManager::getOcc(const Eigen::Vector3d &pos) const
{
  return cur_->getInflateOccupancy(pos);
  // if (map_use_ == SMALL)
  //   return sml_->getInflateOccupancy(pos);
  // else
  //   return big_->getDistance(pos) <= 0;
}

#endif
