#include "plan_env/grid_map.h"
#include <ros/time.h>

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  // double x_size, y_size, z_size;
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);
  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 0.25);

  // node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range3d_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range3d_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range3d_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("grid_map/enable_virtual_wall", mp_.enable_virtual_walll_, false);
  node_.param("grid_map/virtual_ceil", mp_.virtual_ceil_, -1.0);
  node_.param("grid_map/virtual_ground", mp_.virtual_ground_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, 5.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, 0.2);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, 2);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/fading_time", mp_.fading_time_, 1000.0);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, 0.1);
  node_.param("grid_map/light_height", mp_.light_height_, 0.1);
  node_.param("grid_map/light_length", mp_.light_length_, 4.5); 
  node_.param("grid_map/brightness_threshold", mp_.brightness_threshold_, 255); 

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/visual_inflat_map_height", mp_.visual_inflat_map_height_, 10.0);

  node_.param("grid_map/hit_accum_max",     mp_.hit_accum_max_,    40);
  node_.param("grid_map/hit_accum_threshold",    mp_.hit_accum_threshold_,   10);

  visPtr_ = std::make_shared<visualization::Visualization>(node_);

  // mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  // if (mp_.inf_grid_ > 4)
  // {
  //   mp_.inf_grid_ = 4;
  //   mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
  //   ROS_WARN("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
  // }
  const int DefaultInfGrid = 3;
  mp_.resolution_ = max(mp_.obstacles_inflation_ / DefaultInfGrid, 0.13);
  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);

  std::cout << "inflation inf grid : " << mp_.inf_grid_ << ", resolution : " << mp_.resolution_ << std::endl;

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;
  md_.rb_size3i_ = 2 * mp_.local_update_range3i_;
  md_.xy_step_ = md_.rb_size3i_(0) * md_.rb_size3i_(1);
  md_.rb_inf_size3i_ = md_.rb_size3i_ + Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);
  md_.xy_step_inf_ = md_.rb_inf_size3i_(0) * md_.rb_inf_size3i_(1);

  cout << "md_.rb_size3i_=" << md_.rb_size3i_.transpose() << " md_.rb_inf_size3i_=" << md_.rb_inf_size3i_.transpose() << endl;

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.max_clear_log_ = mp_.clamp_min_log_ + 1e-5;
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "occupancy log: " << mp_.min_occupancy_log_ << endl;
  cout << "clear log: " << mp_.max_clear_log_ << endl;

  // initialize data buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  md_.map_voxel_num_ = buffer_size;
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) * (map_voxel_num3i(1) + 2 * mp_.inf_grid_) * (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
  md_.rb_orig3i_ = Eigen::Vector3i(0, 0, 0);
  md_.rb_inf_orig3i_ = Eigen::Vector3i(0, 0, 0);

  md_.occ_buf_            = vector<float>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occ_buf_output_     = vector<float>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occ_buf_inf_        = vector<int16_t>(buffer_inf_size, GRID_MAP_UNKNOWN_FLAG);
  md_.occ_buf_inf_output_ = vector<int16_t>(buffer_inf_size, GRID_MAP_UNKNOWN_FLAG);
  md_.dist_buf_           = vector<double>(buffer_size, GRID_MAP_UNKNOWN_ESDF_FLAG); // must be a big value!
  md_.tmp_buf0_           = vector<double>(buffer_size, 0.0);
  md_.tmp_buf1_           = vector<double>(buffer_size, 0.0);
  md_.tmp_buf2_           = vector<double>(buffer_size, 0.0);
  md_.tmp_buf3_           = vector<double>(buffer_size, 0.0);
#if COMPUTE_NEGTIVE_EDT
  md_.dist_buf_neg_ = vector<double>(buffer_size, GRID_MAP_UNKNOWN_ESDF_FLAG); // must be a big value!
  md_.dist_buf_all_ = vector<double>(buffer_size, GRID_MAP_UNKNOWN_ESDF_FLAG); // must be a big value!
  md_.occ_buf_neg_ = vector<int8_t>(buffer_size, 0);
#endif

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.hit_accum_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.cache_voxel_ = vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));

  md_.raycast_num_ = 0;
  md_.proj_points_cnt_ = 0;
  md_.cache_voxel_cnt_ = 0;

  md_.finish_occupancy_ = true;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_.fromSec(0);
  md_.flag_have_ever_received_depth_ = false;
  md_.flag_have_ever_received_pc_    = false;
  md_.cam2body_ << 0.0,  0.0, 1.0, 0.05,
                  -1.0,  0.0, 0.0, 0.05,
                   0.0, -1.0, 0.0, 0.045,
                   0.0,  0.0, 0.0, 1.0;
  md_.target_pos_update_=false;

  md_.target_pos_update_ = false;

  /* init callback */
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 3));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); // sub
  gray_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/camera/infra1/image_rect_raw", 3));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    ROS_ERROR("Do NOT support POSE_STAMPED input because of the added gray image check in updateOccupancy() !!!");
    // pose_sub_.reset(
    //     new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    // sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
    //     SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    // sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_,  *gray_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2, _3));
  }

  // use odometry and point cloud
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);

  std::thread vis_thread(GridMap::visCallback, this);
  vis_thread.detach();

  if (mp_.fading_time_ > 0)
  {
    std::thread fading_thread(GridMap::fadingCallback, this);
    fading_thread.detach();
  }

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);
  map_unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_unknown", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("grid_map/update_range", 10);
  filterlight_pub_ = node_.advertise<sensor_msgs::Image>("grid_map/filter_depth", 10);
}

void GridMap::paramAdjust(double reso, double rangex, double rangey, double rangez, bool esdf_en, bool inf_en, bool multi_thread, std::string suffix)
{
  mtx_.lock();
  mtx_vis_.lock();

  mp_.resolution_ = reso;
  mp_.local_update_range3d_(0) = rangex;
  mp_.local_update_range3d_(1) = rangey;
  mp_.local_update_range3d_(2) = rangez;
  mp_.esdf_enable_ = esdf_en;

  mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
  if (mp_.inf_grid_ > 4)
  {
    mp_.inf_grid_ = 4;
    mp_.resolution_ = mp_.obstacles_inflation_ / mp_.inf_grid_;
    ROS_WARN("Inflation is too big, which will cause siginificant computation! Resolution enalrged to %f automatically.", mp_.resolution_);
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
  mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.resolution_;
  md_.rb_size3i_ = 2 * mp_.local_update_range3i_;
  md_.xy_step_ = md_.rb_size3i_(0) * md_.rb_size3i_(1);
  md_.rb_inf_size3i_ = md_.rb_size3i_ + Eigen::Vector3i(2 * mp_.inf_grid_, 2 * mp_.inf_grid_, 2 * mp_.inf_grid_);
  md_.xy_step_inf_ = md_.rb_inf_size3i_(0) * md_.rb_inf_size3i_(1);

  cout << "paramAdjust: md_.rb_size3i_=" << md_.rb_size3i_.transpose() << " md_.rb_inf_size3i_=" << md_.rb_inf_size3i_.transpose() << endl;

  // initialize data buffers
  Eigen::Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
  int buffer_size = map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
  int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) * (map_voxel_num3i(1) + 2 * mp_.inf_grid_) * (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
  md_.rb_orig3i_ = Eigen::Vector3i(0, 0, 0);
  md_.rb_inf_orig3i_ = Eigen::Vector3i(0, 0, 0);

  md_.occ_buf_            = vector<float>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occ_buf_output_     = vector<float>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occ_buf_inf_        = vector<int16_t>(buffer_inf_size, GRID_MAP_UNKNOWN_FLAG);
  md_.occ_buf_inf_output_ = vector<int16_t>(buffer_inf_size, GRID_MAP_UNKNOWN_FLAG);
  md_.dist_buf_           = vector<double>(buffer_size, GRID_MAP_UNKNOWN_ESDF_FLAG); // must be a big value!
  md_.tmp_buf0_           = vector<double>(buffer_size, 0.0);
  md_.tmp_buf1_           = vector<double>(buffer_size, 0.0);
  md_.tmp_buf2_           = vector<double>(buffer_size, 0.0);
  md_.tmp_buf3_           = vector<double>(buffer_size, 0.0);
#if COMPUTE_NEGTIVE_EDT
  md_.dist_buf_neg_ = vector<double>(buffer_size, GRID_MAP_UNKNOWN_ESDF_FLAG); // must be a big value!
  md_.dist_buf_all_ = vector<double>(buffer_size, GRID_MAP_UNKNOWN_ESDF_FLAG); // must be a big value!
  md_.occ_buf_neg_ = vector<int8_t>(buffer_size, 0);
#endif

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.hit_accum_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);
  md_.cache_voxel_ = vector<Eigen::Vector3i>(buffer_size, Eigen::Vector3i(0, 0, 0));

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy" + suffix, 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate" + suffix, 10);
  map_unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_unknown" + suffix, 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/esdf" + suffix, 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("grid_map/update_range" + suffix, 10);

  mp_.name_ = suffix;

  // mp_.multi_thread_ = multi_thread;
  mp_.have_initialized_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.last_occ_update_time_.fromSec(0);
  md_.flag_have_ever_received_depth_ = false;
  md_.flag_have_ever_received_pc_    = false;

  mtx_.unlock();
  mtx_vis_.unlock();
}

void GridMap::updateOccupancy(void *obj)
{
  GridMap *map = reinterpret_cast<GridMap *>(obj);

  // cout << map->mp_.name_.substr(0, 1) << "1" << flush;

  /* update occupancy */
  ros::Time t1, t2, t3, t4, t5, t6;
  t1 = ros::Time::now();
  map->mtx_.lock();
  map->md_.finish_occupancy_ = false;

  map->moveRingBuffer();
  t2 = ros::Time::now();

  map->projectDepthImage();
  t3 = ros::Time::now();

  // printf("Raycast Begin===========================================================\n");

  if (map->md_.proj_points_cnt_ > 0)
  {
    map->raycastProcess();
    t4 = ros::Time::now();

    map->clearAndInflateLocalMap(); // md_.occ_buf_inf_ is required for hysteresis loop
    map->CopyToOutputMap();
    t5 = ros::Time::now();

    if (map->mp_.esdf_enable_)
      map->updateESDF3d();
    t6 = ros::Time::now();

    if (map->mp_.show_occ_time_)
    {
      // cout << setprecision(7);
      // cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << " t5=" << (t5 - t4).toSec() << endl;

      map->ts_.raycasttime += (t4 - t3).toSec();
      map->ts_.max_raycasttime = max(map->ts_.max_raycasttime, (t4 - t3).toSec());
      map->ts_.inflationtime += (t5 - t4).toSec();
      map->ts_.max_inflationtime = max(map->ts_.max_inflationtime, (t5 - t4).toSec());
      map->ts_.esdftime += (t6 - t5).toSec();
      map->ts_.max_esdftime = max(map->ts_.max_esdftime, (t6 - t5).toSec());
      ++map->ts_.updatetimes;

      printf("[%s]Raycast(ms): cur t = %lf, avg t = %lf, max t = %lf\n",
             map->mp_.name_.c_str(), (t4 - t3).toSec() * 1000, map->ts_.raycasttime / map->ts_.updatetimes * 1000, map->ts_.max_raycasttime * 1000);
      printf("[%s]Infaltion(ms): cur t = %lf, avg t = %lf, max t = %lf\n",
             map->mp_.name_.c_str(), (t5 - t4).toSec() * 1000, map->ts_.inflationtime / map->ts_.updatetimes * 1000, map->ts_.max_inflationtime * 1000);
      if (map->mp_.esdf_enable_)
      {
        printf("[%s]ESDF(ms): cur t = %lf, avg t = %lf, max t = %lf\n", map->mp_.name_.c_str(), (t6 - t5).toSec() * 1000,
               map->ts_.esdftime / map->ts_.updatetimes * 1000, map->ts_.max_esdftime * 1000);
      }
    }
  }

  // cout << " MP" << map->mp_.name_.substr(0, 1) << setprecision(3) << (ros::Time::now() - map->md_.last_occ_update_time_).toSec() * 1000 << flush;

  map->md_.flag_have_ever_received_depth_ = true;
  map->md_.last_occ_update_time_ = ros::Time::now();
  map->md_.finish_occupancy_ = true;
  map->mtx_.unlock();

  // cout << map->mp_.name_.substr(0, 1) << "2" << flush;
}

// 带raycast的点云输入处理
void GridMap::updateOccupancyPC(void *obj, const sensor_msgs::PointCloud2ConstPtr &img)
{
  GridMap *map = reinterpret_cast<GridMap *>(obj);

  /* update occupancy */
  ros::Time t1, t2, t3, t4, t5, t6;
  t1 = ros::Time::now();
  map->md_.finish_occupancy_ = false;

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(map->md_.camera_pos_(0)) || isnan(map->md_.camera_pos_(1)) || isnan(map->md_.camera_pos_(2)))
    return;

  map->mtx_.lock();
  map->moveRingBuffer();
  t2 = ros::Time::now();

  map->projectPC(latest_cloud);
  t3 = ros::Time::now();

  if (map->md_.proj_points_cnt_ > 0)
  {
    map->raycastProcess();
    t4 = ros::Time::now();

    map->clearAndInflateLocalMap(); // md_.occ_buf_inf_ is required for hysteresis loop
    map->CopyToOutputMap();
    t5 = ros::Time::now();

    if (map->mp_.esdf_enable_)
      map->updateESDF3d();
    t6 = ros::Time::now();

    if (map->mp_.show_occ_time_)
    {
      // cout << setprecision(7);
      // cout << "t2=" << (t2 - t1).toSec() << " t3=" << (t3 - t2).toSec() << " t4=" << (t4 - t3).toSec() << " t5=" << (t5 - t4).toSec() << endl;

      map->ts_.raycasttime += (t4 - t3).toSec();
      map->ts_.max_raycasttime = max(map->ts_.max_raycasttime, (t4 - t3).toSec());
      map->ts_.inflationtime += (t5 - t4).toSec();
      map->ts_.max_inflationtime = max(map->ts_.max_inflationtime, (t5 - t4).toSec());
      map->ts_.esdftime += (t6 - t5).toSec();
      map->ts_.max_esdftime = max(map->ts_.max_esdftime, (t6 - t5).toSec());
      ++map->ts_.updatetimes;

      printf("[%s]Raycast(ms): cur t = %lf, avg t = %lf, max t = %lf\n",
             map->mp_.name_.c_str(), (t4 - t3).toSec() * 1000, map->ts_.raycasttime / map->ts_.updatetimes * 1000, map->ts_.max_raycasttime * 1000);
      printf("[%s]Infaltion(ms): cur t = %lf, avg t = %lf, max t = %lf\n",
             map->mp_.name_.c_str(), (t5 - t4).toSec() * 1000, map->ts_.inflationtime / map->ts_.updatetimes * 1000, map->ts_.max_inflationtime * 1000);
      if (map->mp_.esdf_enable_)
      {
        printf("[%s]ESDF(ms): cur t = %lf, avg t = %lf, max t = %lf\n", map->mp_.name_.c_str(), (t6 - t5).toSec() * 1000,
               map->ts_.esdftime / map->ts_.updatetimes * 1000, map->ts_.max_esdftime * 1000);
      }
    }
  }

  // cout << " MP" << map->mp_.name_.substr(0, 1) << setprecision(3) << (ros::Time::now() - map->md_.last_occ_update_time_).toSec() * 1000 << flush;

  map->md_.flag_have_ever_received_pc_ = true;
  map->md_.last_occ_update_time_ = ros::Time::now();
  map->md_.finish_occupancy_ = true;
  map->mtx_.unlock();

  // cout << map->mp_.name_.substr(0, 1) << "2" << flush;
}


// 不带raycast的点云输入处理 no use!
void GridMap::processCloud(void *obj, const sensor_msgs::PointCloud2ConstPtr &img)
{
  GridMap *map = reinterpret_cast<GridMap *>(obj);

  /* update occupancy */
  ros::Time t1, t2, t3, t4, t5, t6;
  t1 = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(map->md_.camera_pos_(0)) || isnan(map->md_.camera_pos_(1)) || isnan(map->md_.camera_pos_(2)))
    return;

  map->mtx_.lock();
  map->moveRingBuffer();

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    if (p3d.array().isNaN().sum())
      continue;

    if (map->isInBuf(p3d))
    {
      /* inflate the point */
      Eigen::Vector3i idx = map->pos2GlobalIdx(p3d);
      int buf_id = map->globalIdx2BufIdx(idx);
      int inf_buf_id = map->globalIdx2InfBufIdx(idx);
      map->md_.occ_buf_[buf_id] = map->mp_.clamp_max_log_;

      if (!isocc2(inf_buf_id, map) && occ_high2(buf_id, map))
      {
        map->changeInfBuf(true, inf_buf_id, idx);
      }

      if (map->mp_.esdf_enable_)
      {
        map->md_.update_range_lb3i_ = map->min3i(map->md_.update_range_lb3i_, idx);
        map->md_.update_range_ub3i_ = map->max3i(map->md_.update_range_ub3i_, idx);
      }
    }
  }
  t2 = ros::Time::now();

  map->CopyToOutputMap();
  t3 = ros::Time::now();

  if (map->mp_.esdf_enable_)
    map->updateESDF3d();
  t4 = ros::Time::now();

  map->mtx_.unlock();

  if (map->mp_.show_occ_time_)
  {
    printf("[%s]FuseCloud(ms): %lf, Infaltion(ms): %lf, ESDF(ms): %lf\n",
           map->mp_.name_.c_str(), (t2 - t1).toSec() * 1000, (t3 - t2).toSec() * 1000, (t4 - t3).toSec() * 1000);
  }
}

void GridMap::visCallback(void *obj)
{
  GridMap *map = reinterpret_cast<GridMap *>(obj);

  while (true)
  {
    int milli_time = 0;
    if (map->mp_.have_initialized_)
    {
      ros::Time t0 = ros::Time::now();
      map->mtx_vis_.lock();
      map->publishMap();
      map->publishMapInflate();
      map->publishESDF();
      map->publishUpdateRange();
      map->mtx_vis_.unlock();
      ros::Time t1 = ros::Time::now();
      milli_time = (int)((t1 - t0).toSec() * 1000);

      if (map->mp_.show_occ_time_)
      {
        printf("[%s]Visualization(ms):%f\n", map->mp_.name_.c_str(), (t1 - t0).toSec() * 1000);
      }
    }

    std::chrono::milliseconds dura(max(100 - milli_time, 1)); // 10Hz
    std::this_thread::sleep_for(dura);
  }
}

void GridMap::fadingCallback(void *obj)
{
  GridMap *map = reinterpret_cast<GridMap *>(obj);
  const int Hz = 2; // must be an intenger

  while (true)
  {
    const float reduce = (map->mp_.clamp_max_log_ - map->mp_.clamp_min_log_) / (map->mp_.fading_time_ * Hz); // function called at 2Hz

    ros::Time t0 = ros::Time::now();
    map->mtx_.lock();
    for (size_t i = 0; i < map->md_.occ_buf_.size(); ++i)
    {
      if (!occ_low2(i, map))
      {
        map->md_.occ_buf_[i] = max(map->md_.occ_buf_[i] - reduce, map->mp_.clamp_min_log_);
        if (occ_low2(i, map))
        {
          Eigen::Vector3i idx = map->BufIdx2GlobalIdx(i);
          int inf_buf_idx = map->globalIdx2InfBufIdx(idx);
          if (isocc2(inf_buf_idx, map))
          {
            map->changeInfBuf(false, inf_buf_idx, idx);
          }
        }
      }
    }
    map->mtx_.unlock();
    ros::Time t1 = ros::Time::now();
    int milli_time = (int)((t1 - t0).toSec() * 1000);

    if (map->mp_.show_occ_time_)
    {
      printf("[%s]Fading(ms):%f\n", map->mp_.name_.c_str(), (t1 - t0).toSec() * 1000);
    }

    std::chrono::milliseconds dura(max(1000 / Hz - milli_time, 1)); // 2Hz
    std::this_thread::sleep_for(dura);
  }
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  // static ros::Time t0;
  // std::cout << std::setprecision(3);
  // std::cout << "CX" << (ros::Time::now() - t0).toSec() * 1000 << std::flush;
  // t0 = ros::Time::now();

  // ROS_WARN_STREAM("t: " << ros::Time::now() << "  " << mp_.name_);

  // cout << " FX" << mp_.name_.substr(0,1) << ros::Time::now().nsec / 1000000 << flush;

  if (md_.finish_occupancy_)
  {
    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    
    // ROS_ERROR_STREAM("img->encoding: " << img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }

    mtx_.lock();
    cv_ptr->image.copyTo(md_.depth_image_);

    if ((int)md_.proj_points_.size() != md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_)
      md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);

    // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

    /* get pose */
    md_.camera_pos_(0) = pose->pose.position.x;
    md_.camera_pos_(1) = pose->pose.position.y;
    md_.camera_pos_(2) = pose->pose.position.z;
    md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                         pose->pose.orientation.y, pose->pose.orientation.z)
                          .toRotationMatrix();
    mtx_.unlock();

    // if (mp_.multi_thread_)
    // {
    std::thread update_occ_thread(GridMap::updateOccupancy, this);
    update_occ_thread.detach();
    // }
    // else
    //   updateOccupancy(this);
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]updateOccupancy not finished! Too long!", mp_.name_.c_str());
  }

  // std::cout << "Cx" << (ros::Time::now() - t0).toSec() * 1000 << std::flush;
  // t0 = ros::Time::now();
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom,
                                const sensor_msgs::ImageConstPtr &gray)
{
  if (md_.finish_occupancy_)
  {
    /* get pose */
    Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                   odom->pose.pose.orientation.x,
                                                   odom->pose.pose.orientation.y,
                                                   odom->pose.pose.orientation.z);
    Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
    Eigen::Matrix4d body2world;
    body2world.block<3, 3>(0, 0) = body_r_m;
    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;
    body2world(3, 3) = 1.0;

    Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
    md_.camera_pos_(0) = cam_T(0, 3);
    md_.camera_pos_(1) = cam_T(1, 3);
    md_.camera_pos_(2) = cam_T(2, 3);
    md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr, gray_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    gray_ptr = cv_bridge::toCvCopy(gray, gray->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }

    mtx_.lock();
    cv_ptr->image.copyTo(md_.depth_image_);
    gray_ptr->image.copyTo(md_.gray_image_);

    if ((int)md_.proj_points_.size() != md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_)
      md_.proj_points_.resize(md_.depth_image_.cols * md_.depth_image_.rows / mp_.skip_pixel_ / mp_.skip_pixel_);
        
    mtx_.unlock();

    // if (mp_.multi_thread_)
    // {
    std::thread update_occ_thread(GridMap::updateOccupancy, this);
    update_occ_thread.detach();
    // }
    // else
    //   updateOccupancy(this);
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "[%s]updateOccupancy not finished! Too long!", mp_.name_.c_str());
  }
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{

  if (md_.flag_have_ever_received_depth_)
  {
    indep_odom_sub_.shutdown();
    return;
  }

  /* get pose */
  mtx_.lock();
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  md_.has_odom_ = true;
  mtx_.unlock();
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  /* Note: no obstalce elimination in this function! */
  if (!md_.has_odom_)
  {
    std::cout << "grid_map: no odom!" << std::endl;
    return;
  }

  // std::thread process_cloud_thread(GridMap::processCloud, this, img);
  std::thread process_cloud_thread(GridMap::updateOccupancyPC, this, img);
  
  process_cloud_thread.detach();
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  mtx_.lock();

  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;

  mtx_.unlock();
}

template <typename F_get_val, typename F_set_val>
void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[md_.rb_size3i_(dim)];        // stores the grid location of the lower parabola
  double z[md_.rb_size3i_(dim) + 1]; // stores the intersection locations as lower-envelope boundaries

  int k = start; // Index of rightmost parabola in lower envelope
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  // compute lower envelope
  for (int q = start + 1; q <= end; q++)
  {
    k++;
    double s;

    do
    {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]); // position of the intersection
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  // fill data
  k = start;
  for (int q = start; q <= end; q++)
  {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void GridMap::updateESDF3d()
{
  Eigen::Vector3i min_esdf = min3i(max3i(md_.update_range_lb3i_ /*- Eigen::Vector3i(5, 5, 5)*/, md_.rb_lb3i_),
                                   pos2GlobalIdx(md_.camera_pos_) - Eigen::Vector3i::Ones());
  Eigen::Vector3i max_esdf = max3i(min3i(md_.update_range_ub3i_ /*+ Eigen::Vector3i(5, 5, 5)*/, md_.rb_ub3i_),
                                   pos2GlobalIdx(md_.camera_pos_) + Eigen::Vector3i::Ones());

  // cout << "min_esdf: " << globalIdx2Pos(min_esdf).transpose() << ", max_esdf: " << globalIdx2Pos(max_esdf).transpose() << endl;

  
  Eigen::Vector3i tmpbuf_min = globalIdx2TmpBuf3dIdx(min_esdf);
  Eigen::Vector3i tmpbuf_max = globalIdx2TmpBuf3dIdx(max_esdf);
  // md_.update_range_lb3i_ = md_.rb_ub3i_; // reset data has been changed to raycastProcess() 
  // md_.update_range_ub3i_ = md_.rb_lb3i_; // reset data

  if ((max_esdf - min_esdf).minCoeff() <= 2)
  {
    Eigen::Vector3i range = (max_esdf - min_esdf);
    ROS_WARN("(max_esdf - min_esdf)=(%d, %d, %d), the update range is too small, return.", range(0), range(1), range(2));
    return;
  }

  /* ========== compute positive DT ========== */
  // ros::Time t1 = ros::Time::now();

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
      {
        md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, y, z)))] =
            isocc(globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z))) ? 0 : std::numeric_limits<double>::max();
      }

  // const double SQRTMAXDOUBLE = sqrt(std::numeric_limits<double>::max() - 1.0); // to avoid inf
  // for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
  //   for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
  //   {
  //     double v0 = md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(x, y, min_esdf[2]))] * mp_.resolution_inv_;
  //     md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, y, min_esdf[2])))] =
  //         v0 < SQRTMAXDOUBLE ? v0 * v0 : std::numeric_limits<double>::max();
  //     double v1 = md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(x, y, max_esdf[2]))] * mp_.resolution_inv_;
  //     md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, y, max_esdf[2])))] =
  //         v1 < SQRTMAXDOUBLE ? v1 * v1 : std::numeric_limits<double>::max();
  //   }
  // for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
  //   for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
  //   {
  //     double v0 = md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(x, min_esdf[1], z))] * mp_.resolution_inv_;
  //     md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, min_esdf[1], z)))] =
  //         v0 < SQRTMAXDOUBLE ? v0 * v0 : std::numeric_limits<double>::max();
  //     double v1 = md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(x, max_esdf[1], z))] * mp_.resolution_inv_;
  //     md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, max_esdf[1], z)))] =
  //         v1 < SQRTMAXDOUBLE ? v1 * v1 : std::numeric_limits<double>::max();
  //   }
  // for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
  //   for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
  //   {
  //     double v0 = md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(min_esdf[0], y, z))] * mp_.resolution_inv_;
  //     md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(min_esdf[0], y, z)))] =
  //         v0 < SQRTMAXDOUBLE ? v0 * v0 : std::numeric_limits<double>::max();
  //     double v1 = md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(max_esdf[0], y, z))] * mp_.resolution_inv_;
  //     md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(max_esdf[0], y, z)))] =
  //         v1 < SQRTMAXDOUBLE ? v1 * v1 : std::numeric_limits<double>::max();
  //   }

  // ros::Time t2 = ros::Time::now();

  for (int x = tmpbuf_min[0]; x <= tmpbuf_max[0]; x++)
  {
    for (int y = tmpbuf_min[1]; y <= tmpbuf_max[1]; y++)
    {
      fillESDF(
          [&](int z)
          {
            return md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))];
          },
          [&](int z, double val)
          { md_.tmp_buf1_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] = val; },
          tmpbuf_min[2],
          tmpbuf_max[2], 2);
    }
  }

  for (int x = tmpbuf_min[0]; x <= tmpbuf_max[0]; x++)
  {
    for (int z = tmpbuf_min[2]; z <= tmpbuf_max[2]; z++)
    {
      fillESDF([&](int y)
               { return md_.tmp_buf1_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))]; },
               [&](int y, double val)
               { md_.tmp_buf2_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] = val; },
               tmpbuf_min[1],
               tmpbuf_max[1], 1);
    }
  }

  for (int y = tmpbuf_min[1]; y <= tmpbuf_max[1]; y++)
  {
    for (int z = tmpbuf_min[2]; z <= tmpbuf_max[2]; z++)
    {
      fillESDF([&](int x)
               { return md_.tmp_buf2_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))]; },
               [&](int x, double val)
               { md_.tmp_buf3_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] = mp_.resolution_ * std::sqrt(val); },
               tmpbuf_min[0],
               tmpbuf_max[0], 0);
    }
  }

  // ros::Time t3 = ros::Time::now();

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
      {
        md_.dist_buf_[globalIdx2BufIdx(Eigen::Vector3i(x, y, z))] =
            md_.tmp_buf3_[TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, y, z)))];
      }

      // ros::Time t4 = ros::Time::now();
      // ROS_ERROR("tcopy=%f, tesdf=%f, all=%f, grids=%d", ((t4 - t3).toSec() + (t2 - t1).toSec()) * 1000, (t3 - t2).toSec() * 1000,
      //           (t4 - t1).toSec() * 1000, (max_esdf(0) - min_esdf[0]) * (max_esdf(1) - min_esdf[1]) * (max_esdf(2) - min_esdf[2]));

#if COMPUTE_NEGTIVE_EDT
  /* ========== compute negative distance ========== */

  for (int x = tmpbuf_min[0]; x <= tmpbuf_max[0]; x++)
  {
    for (int y = tmpbuf_min[1]; y <= tmpbuf_max[1]; y++)
    {
      fillESDF(
          [&](int z)
          {
            return md_.tmp_buf0_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] > 1e-5
                       ? 0
                       : std::numeric_limits<double>::max();
          },
          [&](int z, double val)
          { md_.tmp_buf1_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] = val; },
          tmpbuf_min[2],
          tmpbuf_max[2], 2);
    }
  }

  for (int x = tmpbuf_min[0]; x <= tmpbuf_max[0]; x++)
  {
    for (int z = tmpbuf_min[2]; z <= tmpbuf_max[2]; z++)
    {
      fillESDF([&](int y)
               { return md_.tmp_buf1_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))]; },
               [&](int y, double val)
               { md_.tmp_buf2_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] = val; },
               tmpbuf_min[1],
               tmpbuf_max[1], 1);
    }
  }

  for (int y = tmpbuf_min[1]; y <= tmpbuf_max[1]; y++)
  {
    for (int z = tmpbuf_min[2]; z <= tmpbuf_max[2]; z++)
    {
      fillESDF([&](int x)
               { return md_.tmp_buf2_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))]; },
               [&](int x, double val)
               {
                 md_.dist_buf_neg_[TmpBuf3dIdx2TmpBuf1dIdx(Eigen::Vector3i(x, y, z))] = mp_.resolution_ * std::sqrt(val);
               },
               tmpbuf_min[0], tmpbuf_max[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z)
      {

        int idx = globalIdx2BufIdx(Eigen::Vector3i(x, y, z));
        md_.dist_buf_all_[idx] = md_.dist_buf_[idx];

        int idx_tmpbuf = TmpBuf3dIdx2TmpBuf1dIdx(globalIdx2TmpBuf3dIdx(Eigen::Vector3i(x, y, z)));
        if (md_.dist_buf_neg_[idx_tmpbuf] > 0.0)
          md_.dist_buf_all_[idx] += (-md_.dist_buf_neg_[idx_tmpbuf] + mp_.resolution_);
      }
#endif
}

void GridMap::moveRingBuffer()
{
  if (!mp_.have_initialized_)
    initMapBoundary();

  Eigen::Vector3i center_new = pos2GlobalIdx(md_.camera_pos_);
  Eigen::Vector3i ringbuffer_lowbound3i_new = center_new - mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_upbound3i_new = center_new + mp_.local_update_range3i_;
  Eigen::Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * mp_.resolution_;
  ringbuffer_upbound3i_new -= Eigen::Vector3i(1, 1, 1); // ub_id(0,0,0) -> ubpos(0.1, 0.1, 0.1)
  md_.tmpbuf_offset_ = center_new - mp_.local_update_range3i_;

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  Eigen::Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
  Eigen::Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_upbound3i_new + inf_grid3i;
  Eigen::Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_upbound3d_new + inf_grid3d;

  if (center_new(0) < md_.center_last3i_(0))
    clearBuffer(0, ringbuffer_upbound3i_new(0));
  if (center_new(0) > md_.center_last3i_(0))
    clearBuffer(1, ringbuffer_lowbound3i_new(0));
  if (center_new(1) < md_.center_last3i_(1))
    clearBuffer(2, ringbuffer_upbound3i_new(1));
  if (center_new(1) > md_.center_last3i_(1))
    clearBuffer(3, ringbuffer_lowbound3i_new(1));
  if (center_new(2) < md_.center_last3i_(2))
    clearBuffer(4, ringbuffer_upbound3i_new(2));
  if (center_new(2) > md_.center_last3i_(2))
    clearBuffer(5, ringbuffer_lowbound3i_new(2));

  for (int i = 0; i < 3; ++i)
  {
    while (md_.rb_orig3i_(i) < md_.rb_lb3i_(i))
    {
      md_.rb_orig3i_(i) += md_.rb_size3i_(i);
    }
    while (md_.rb_orig3i_(i) > md_.rb_ub3i_(i))
    {
      md_.rb_orig3i_(i) -= md_.rb_size3i_(i);
    }

    while (md_.rb_inf_orig3i_(i) < md_.rb_inf_lb3i_(i))
    {
      md_.rb_inf_orig3i_(i) += md_.rb_inf_size3i_(i);
    }
    while (md_.rb_inf_orig3i_(i) > md_.rb_inf_ub3i_(i))
    {
      md_.rb_inf_orig3i_(i) -= md_.rb_inf_size3i_(i);
    }
  }

  md_.center_last3i_ = center_new;
  md_.rb_lb3i_ = ringbuffer_lowbound3i_new;
  md_.rb_lb3d_ = ringbuffer_lowbound3d_new;
  md_.rb_ub3i_ = ringbuffer_upbound3i_new;
  md_.rb_ub3d_ = ringbuffer_upbound3d_new;
  md_.rb_inf_lb3i_ = ringbuffer_inf_lowbound3i_new;
  md_.rb_inf_lb3d_ = ringbuffer_inf_lowbound3d_new;
  md_.rb_inf_ub3i_ = ringbuffer_inf_upbound3i_new;
  md_.rb_inf_ub3d_ = ringbuffer_inf_upbound3d_new;
}

void GridMap::projectDepthImage()
{
  md_.proj_points_cnt_ = 0;

  uint16_t *row_ptr;
  uint8_t *gray_row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  vector<Eigen::Vector3d> pj_pts;

  int filteru=getu(rows);//filter 

  // force to use the depth filter!
  for (int v = filteru; v < rows - mp_.depth_filter_margin_; v += skip_pix)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v);
    gray_row_ptr = md_.gray_image_.ptr<uint8_t>(v);

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += skip_pix)
    {

      Eigen::Vector3d proj_pt;
      depth = (*row_ptr) / mp_.k_depth_scaling_factor_;
      row_ptr = row_ptr + mp_.skip_pixel_;

      uint8_t gray = *gray_row_ptr;
      gray_row_ptr = gray_row_ptr + mp_.skip_pixel_;

      //! brightness filer
      if (int(gray) > mp_.brightness_threshold_)
        continue;

      //! depth filter
      if (*row_ptr == 0) 
      {
        // depth = mp_.depth_filter_maxdist_ + 0.1;
        continue; //! 深度图中没有距离的格子应该直接被continue掉
      } 
      else if (depth < mp_.depth_filter_mindist_) 
      {
        continue;
      } 
      else if (depth > mp_.depth_filter_maxdist_) 
      {
        depth = mp_.depth_filter_maxdist_ + 0.1;
      }

      // project to world frame
      proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
      proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
      proj_pt(2) = depth;

      proj_pt = camera_r * proj_pt + md_.camera_pos_;
      md_.proj_points_[md_.proj_points_cnt_++] = proj_pt;
      pj_pts.push_back(proj_pt);
    }
  }

  // Below for vis
  visPtr_->visualize_pointcloud(pj_pts, "project_pts");

  if ( filterlight_pub_.getNumSubscribers() >= 1 )
  {
    cv::Mat depth_image_filter(md_.depth_image_.rows, md_.depth_image_.cols, CV_16UC1);
    depth_image_filter.setTo(cv::Scalar(0));
    uint16_t *debug_row_ptr;
    for (int v = filteru; v < rows - mp_.depth_filter_margin_; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);
      gray_row_ptr = md_.gray_image_.ptr<uint8_t>(v);
      debug_row_ptr = depth_image_filter.ptr<uint16_t>(v);

      for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += skip_pix, debug_row_ptr += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        uint16_t depth_int = *row_ptr;
        depth = (*row_ptr) / mp_.k_depth_scaling_factor_;
        row_ptr = row_ptr + mp_.skip_pixel_;

        uint8_t gray = *gray_row_ptr;
        gray_row_ptr = gray_row_ptr + mp_.skip_pixel_;

        //! brightness filer
        if (int(gray) > mp_.brightness_threshold_)
          continue;

        //! depth filter
        if (*row_ptr == 0) 
        {
          // depth = mp_.depth_filter_maxdist_ + 0.1;
          continue; //! 深度图中没有距离的格子应该直接被continue掉
        } 
        else if (depth < mp_.depth_filter_mindist_) 
        {
          continue;
        } 
        else if (depth > mp_.depth_filter_maxdist_) 
        {
          depth = mp_.depth_filter_maxdist_ + 0.1;
        }

        *debug_row_ptr = depth_int;
      }
    }

    // Convert cv::Mat to sensor_msgs::Image
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now(); // Assign the current time to the header
    cv_image.header.frame_id =mp_.frame_id_; // Replace with your actual frame ID
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Specify the encoding
    cv_image.image = depth_image_filter; // Assign the cv::Mat to the cv_bridge image
    // Convert CvImage to sensor_msgs::Image
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    // Publish the depth image
    filterlight_pub_.publish(ros_image);
  }
}

void GridMap::projectPC(pcl::PointCloud<pcl::PointXYZ> &pc)
{
  md_.proj_points_.resize(pc.points.size());

  md_.proj_points_cnt_ = 0;
  int skip_pix = mp_.skip_pixel_;

  for (int i = 0; i < pc.points.size(); i += 1)
  {
    Eigen::Vector3d proj_pt;
    proj_pt(0) = pc.points[i].x;
    proj_pt(1) = pc.points[i].y;
    proj_pt(2) = pc.points[i].z;

    if ((proj_pt - md_.camera_pos_).norm() > mp_.depth_filter_maxdist_)
    {
      proj_pt = md_.camera_pos_ + (mp_.depth_filter_maxdist_ + 0.1) * (proj_pt - md_.camera_pos_).normalized();
    }

    md_.proj_points_[md_.proj_points_cnt_++] = proj_pt;
  }
}


void GridMap::raycastProcess()
{
  md_.cache_voxel_cnt_ = 0;

  ros::Time t1, t2, t3;

  md_.raycast_num_ += 1;

  RayCaster raycaster;
  Eigen::Vector3d ray_pt, end_p, origin_p;
  origin_p = md_.camera_pos_;

  int pts_num = 0;

  md_.update_range_lb3i_ = md_.rb_ub3i_; // reset data
  md_.update_range_ub3i_ = md_.rb_lb3i_; // reset data

  t1 = ros::Time::now();
  for (int i = 0; i < md_.proj_points_cnt_; ++i)
  {
    int vox_idx;
    end_p = md_.proj_points_[i];
    pts_num++;

    // set flag for projected point
    //! 1. Out of the Map
    if (!isInBuf(end_p))
    {
      end_p = closetPointInMap(end_p, origin_p);
      vox_idx = setCacheOccupancy(end_p, 0);
    }
    //! 2. Inside the Map
    else
    {
      //! 2.1. Inside the sensor range
      if((end_p - origin_p).norm() < mp_.depth_filter_maxdist_){
        vox_idx = setCacheOccupancy(end_p, 1);
      }
      //! 2.2. Out of the sensor range
      else{
        end_p = origin_p + (end_p - origin_p).normalized() * mp_.depth_filter_maxdist_;
        vox_idx = setCacheOccupancy(end_p, 0);
      }
    }

    // raycasting between camera center and point

    // if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(end_p / mp_.resolution_, origin_p / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.resolution_;

      pts_num++;
      vox_idx = setCacheOccupancy(tmp, 0);

      // if (vox_idx != INVALID_IDX)
      // {
      if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
      {
        break;
      }
      else
      {
        md_.flag_traverse_[vox_idx] = md_.raycast_num_;
      }
      // }
    }
  }

  double person_half_x = 0.3;
  double person_half_y = 0.3;
  double person_half_z = 1.0;
  mp_.clear_target_pos_num_ = 50;
  int person_inf_x = ceil(person_half_x / mp_.resolution_);
  int person_inf_y = ceil(person_half_y / mp_.resolution_);
  int person_inf_z = ceil(person_half_z / mp_.resolution_);

  if (md_.target_pos_update_)
  {
    Eigen::Vector3i target_id = pos2GlobalIdx(md_.target_pos_);

    for (int x = -person_inf_x; x <= person_inf_x; ++x)
      for (int y = -person_inf_y; y <= person_inf_y; ++y)
        for (int z = -person_inf_z; z <= person_inf_z; ++z)
        {
          Eigen::Vector3i id_inf(target_id + Eigen::Vector3i(x, y, z));
          Eigen::Vector3d pos_inf = globalIdx2Pos(id_inf);

          //! 1. Inside the Map
          if (isInBuf(pos_inf))
          {
            for (int i = 0; i < mp_.clear_target_pos_num_; i++)
            {
              setCacheOccupancy(pos_inf, 0);
            }
            int idx_ctns = globalIdx2BufIdx(id_inf);
            md_.count_hit_[idx_ctns] = 0;
          }
        }
  }

  t2 = ros::Time::now();

  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {
    auto idx = md_.cache_voxel_[i];

    if (mp_.esdf_enable_) // determine esdf update range
    {
      md_.update_range_lb3i_ = min3i(md_.update_range_lb3i_, idx);
      md_.update_range_ub3i_ = max3i(md_.update_range_ub3i_, idx);
    }

    int idx_ctns = globalIdx2BufIdx(idx);

    if (md_.count_hit_[idx_ctns] > 0)
    {
      md_.hit_accum_[idx_ctns] += md_.count_hit_[idx_ctns];
      if ( md_.hit_accum_[idx_ctns] > (short)mp_.hit_accum_max_ )
        md_.hit_accum_[idx_ctns] = (short)mp_.hit_accum_max_;
    }
    else if ( md_.hit_accum_[idx_ctns] > 0 )
    {
      md_.hit_accum_[idx_ctns] -= (md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns]);
      if ( md_.hit_accum_[idx_ctns] < 0 )
        md_.hit_accum_[idx_ctns] = 0;
    }

    float log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] || 
        md_.hit_accum_[idx_ctns] > (short)mp_.hit_accum_threshold_ ? 
        mp_.prob_hit_log_ : mp_.prob_miss_log_;

    // float log_odds_update =
    //     md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // if (log_odds_update >= 0 && md_.occ_buf_[idx_ctns] >= mp_.clamp_max_log_)    // 为啥要加这个判断，直接更新不就行了，降低代码可读性还导致frontier bug TMD
    // {
    //   continue;
    // }
    // else if (log_odds_update <= 0 && md_.occ_buf_[idx_ctns] <= mp_.clamp_min_log_)
    // {
    //   continue;
    // }

    md_.occ_buf_[idx_ctns] =
        std::min(std::max(md_.occ_buf_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }

  t3 = ros::Time::now();

  if (mp_.show_occ_time_)
  {
    ROS_WARN("Raycast time: t2-t1=%f, t3-t2=%f, pts_num=%d", (t2 - t1).toSec(), (t3 - t2).toSec(), pts_num);
  }
}

void GridMap::clearAndInflateLocalMap()
{
  for (int i = 0; i < md_.cache_voxel_cnt_; ++i)
  {
    Eigen::Vector3i idx = md_.cache_voxel_[i];
    int buf_id = globalIdx2BufIdx(idx);
    int inf_buf_id = globalIdx2InfBufIdx(idx);

    if (md_.occ_buf_inf_[inf_buf_id] == GRID_MAP_UNKNOWN_FLAG)
      md_.occ_buf_inf_[inf_buf_id] = 0;

    if (!isocc(inf_buf_id) && occ_high(buf_id))
      changeInfBuf(true, inf_buf_id, idx);

    if (isocc(inf_buf_id) && occ_low(buf_id))
      changeInfBuf(false, inf_buf_id, idx);
  }
}

void GridMap::CopyToOutputMap()
{
  if (mtx_memcpy_.try_lock())
  {
    std::memcpy(md_.occ_buf_output_.data(), md_.occ_buf_.data(), sizeof(float) * md_.occ_buf_.size());
    mtx_memcpy_.unlock();
  }
  else
  {
    ROS_ERROR("[%s]The planner occupies use occ_map too much time!", mp_.name_.c_str());
  }
  if (mtx_memcpy_inf_.try_lock())
  {
    std::memcpy(md_.occ_buf_inf_output_.data(), md_.occ_buf_inf_.data(), sizeof(int16_t) * md_.occ_buf_inf_.size());
    mtx_memcpy_inf_.unlock();
  }
  else
  {
    ROS_ERROR("[%s]The planner occupies use occ_inf_map too much time!", mp_.name_.c_str());
  }
}

void GridMap::LockCopyToOutputInfMap(bool lock)
{
  if (lock)
    mtx_memcpy_.lock();
  else
    mtx_memcpy_.unlock();
}

void GridMap::LockCopyToOutputAllMap(bool lock)
{
  if (lock)
  {
    mtx_memcpy_.lock();
    mtx_memcpy_inf_.lock();
  }
  else
  {
    mtx_memcpy_.unlock();
    mtx_memcpy_inf_.unlock();
  }
}

void GridMap::initMapBoundary()
{
  mp_.have_initialized_ = true;

  md_.center_last3i_ = pos2GlobalIdx(md_.camera_pos_);
  md_.tmpbuf_offset_ = md_.center_last3i_ - mp_.local_update_range3i_;

  md_.rb_lb3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
  md_.rb_lb3d_ = md_.rb_lb3i_.cast<double>() * mp_.resolution_;
  md_.rb_ub3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
  md_.rb_ub3d_ = md_.rb_ub3i_.cast<double>() * mp_.resolution_;
  md_.rb_ub3i_ -= Eigen::Vector3i(1, 1, 1);

  const Eigen::Vector3i inf_grid3i(mp_.inf_grid_, mp_.inf_grid_, mp_.inf_grid_);
  const Eigen::Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.resolution_;
  md_.rb_inf_lb3i_ = md_.rb_lb3i_ - inf_grid3i;
  md_.rb_inf_lb3d_ = md_.rb_lb3d_ - inf_grid3d;
  md_.rb_inf_ub3i_ = md_.rb_ub3i_ + inf_grid3i;
  md_.rb_inf_ub3d_ = md_.rb_ub3d_ + inf_grid3d;

  // cout << "md_.rb_lb3i_=" << md_.rb_lb3i_.transpose() << " md_.rb_lb3d_=" << md_.rb_lb3d_.transpose() << " md_.rb_ub3i_=" << md_.rb_ub3i_.transpose() << " md_.rb_ub3d_=" << md_.rb_ub3d_.transpose() << endl;

  for (int i = 0; i < 3; ++i)
  {
    while (md_.rb_orig3i_(i) < md_.rb_lb3i_(i))
    {
      md_.rb_orig3i_(i) += md_.rb_size3i_(i);
    }
    while (md_.rb_orig3i_(i) > md_.rb_ub3i_(i))
    {
      md_.rb_orig3i_(i) -= md_.rb_size3i_(i);
    }

    while (md_.rb_inf_orig3i_(i) < md_.rb_inf_lb3i_(i))
    {
      md_.rb_inf_orig3i_(i) += md_.rb_inf_size3i_(i);
    }
    while (md_.rb_inf_orig3i_(i) > md_.rb_inf_ub3i_(i))
    {
      md_.rb_inf_orig3i_(i) -= md_.rb_inf_size3i_(i);
    }
  }

#if GRID_MAP_NEW_PLATFORM_TEST
  testIndexingCost();
#endif
}

void GridMap::clearBuffer(char casein, int bound)
{
  for (int x = (casein == 0 ? bound : md_.rb_lb3i_(0)); x <= (casein == 1 ? bound : md_.rb_ub3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.rb_lb3i_(1)); y <= (casein == 3 ? bound : md_.rb_ub3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.rb_lb3i_(2)); z <= (casein == 5 ? bound : md_.rb_ub3i_(2)); ++z)
      {
        Eigen::Vector3i id_global(x, y, z);
        int id_buf = globalIdx2BufIdx(id_global);
        int id_buf_inf = globalIdx2InfBufIdx(id_global);
        md_.count_hit_[id_buf] = 0;
        md_.count_hit_and_miss_[id_buf] = 0;
        md_.hit_accum_[id_buf] = 0;
        md_.flag_traverse_[id_buf] = md_.raycast_num_;
        md_.flag_rayend_[id_buf] = md_.raycast_num_;
        md_.occ_buf_[id_buf] = mp_.clamp_min_log_ - mp_.unknown_flag_;
#if COMPUTE_NEGTIVE_EDT
        md_.dist_buf_all_[id_buf] = GRID_MAP_UNKNOWN_ESDF_FLAG;
#else
        md_.dist_buf_[id_buf] = GRID_MAP_UNKNOWN_ESDF_FLAG;
#endif

        if (md_.occ_buf_inf_[id_buf_inf] > GRID_MAP_OBS_FLAG)
        {
          changeInfBuf(false, id_buf_inf, id_global);
        }
      }

#if GRID_MAP_NEW_PLATFORM_TEST
  for (int x = (casein == 0 ? bound : md_.rb_lb3i_(0)); x <= (casein == 1 ? bound : md_.rb_ub3i_(0)); ++x)
    for (int y = (casein == 2 ? bound : md_.rb_lb3i_(1)); y <= (casein == 3 ? bound : md_.rb_ub3i_(1)); ++y)
      for (int z = (casein == 4 ? bound : md_.rb_lb3i_(2)); z <= (casein == 5 ? bound : md_.rb_ub3i_(2)); ++z)
      {
        Eigen::Vector3i id_global_inf_clr((casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                          (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                          (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
        int id_buf_inf_clr = globalIdx2InfBufIdx(id_global_inf_clr);
        if (md_.occ_buf_inf_[id_buf_inf_clr] != 0 && md_.occ_buf_inf_[id_buf_inf_clr] != GRID_MAP_UNKNOWN_FLAG)
        {
          ROS_ERROR("Here should be 0!!! md_.occ_buf_inf_[id_buf_inf_clr]=%d", md_.occ_buf_inf_[id_buf_inf_clr]);
        }
      }
#endif

  for (int x = (casein == 0 ? bound + mp_.inf_grid_ : md_.rb_inf_lb3i_(0)); x <= (casein == 1 ? bound - mp_.inf_grid_ : md_.rb_inf_ub3i_(0)); ++x)
    for (int y = (casein == 2 ? bound + mp_.inf_grid_ : md_.rb_inf_lb3i_(1)); y <= (casein == 3 ? bound - mp_.inf_grid_ : md_.rb_inf_ub3i_(1)); ++y)
      for (int z = (casein == 4 ? bound + mp_.inf_grid_ : md_.rb_inf_lb3i_(2)); z <= (casein == 5 ? bound - mp_.inf_grid_ : md_.rb_inf_ub3i_(2)); ++z)
      {
        int id_buf_inf_clr = globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z));
        if (unlikely(md_.occ_buf_inf_[id_buf_inf_clr] > 0))
          ROS_ERROR("md_.occ_buf_inf_[id_buf_inf_clr]=%d", md_.occ_buf_inf_[id_buf_inf_clr]);
        md_.occ_buf_inf_[id_buf_inf_clr] = GRID_MAP_UNKNOWN_FLAG;
      }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = md_.rb_ub3d_ - camera_pt;
  Eigen::Vector3d min_tc = md_.rb_lb3d_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

int GridMap::getu(int rows)
{
  double r21 = md_.camera_r_m_(1,2);
  double r11 = md_.camera_r_m_(0,2);
  double yaw = atan2(r21, r11);
  yaw = -yaw;
  
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q(yawAngle);
  Eigen::Matrix3d r_yaw = q.toRotationMatrix();
  Eigen::Vector3d pt_world= r_yaw.transpose() * Eigen::Vector3d(mp_.light_length_, 0, mp_.light_height_ - md_.camera_pos_[2]) + md_.camera_pos_; 
  Eigen::Matrix3d camera_r_inv=md_.camera_r_m_.transpose();
  Eigen::Vector3d pt_reproj = camera_r_inv * (pt_world - md_.camera_pos_);

  double uu = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;
  
  if(uu > 0 && uu < rows)
    if (uu < mp_.depth_filter_margin_)
      return mp_.depth_filter_margin_;
    else
      return int(uu);
  else
    return 0;

}

void GridMap::publishMap() const
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  // Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.rb_lb3d_(2), mp_.virtual_ground_) : md_.rb_lb3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.rb_ub3d_(2), mp_.virtual_ceil_) : md_.rb_ub3d_(2);
  if (md_.rb_ub3d_(0) - md_.rb_lb3d_(0) > mp_.resolution_ && (md_.rb_ub3d_(1) - md_.rb_lb3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.rb_lb3d_(0) + mp_.resolution_ / 2; xd <= md_.rb_ub3d_(0); xd += mp_.resolution_)
      for (double yd = md_.rb_lb3d_(1) + mp_.resolution_ / 2; yd <= md_.rb_ub3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd <= ubz; zd += mp_.resolution_)
        {
          // Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);
          // if (heading.dot(relative_dir.normalized()) > 0.5)
          // {
          // if (occ_high(globalIdx2BufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))))
          //   cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          if (isocc(globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))))
            cloud.push_back(pcl::PointXYZ(xd, yd, zd));
          // }
        }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate() const
{

  bool inf_sub = map_inf_pub_.getNumSubscribers() > 0;
  bool uk_sub = map_unknown_pub_.getNumSubscribers() > 0;

  if (!inf_sub && !uk_sub)
    return;

  // Eigen::Vector3d heading = (md_.camera_r_m_ * md_.cam2body_.block<3, 3>(0, 0).transpose()).block<3, 1>(0, 0);
  pcl::PointCloud<pcl::PointXYZ> cloud_inf, cloud_uk;
  double lbz = mp_.enable_virtual_walll_ ? max(md_.rb_inf_lb3d_(2), mp_.virtual_ground_) : md_.rb_inf_lb3d_(2);
  double ubz = mp_.enable_virtual_walll_ ? min(md_.rb_inf_ub3d_(2), mp_.virtual_ceil_) : md_.rb_inf_ub3d_(2);
  if (md_.rb_inf_ub3d_(0) - md_.rb_inf_lb3d_(0) > mp_.resolution_ &&
      (md_.rb_inf_ub3d_(1) - md_.rb_inf_lb3d_(1)) > mp_.resolution_ && (ubz - lbz) > mp_.resolution_)
    for (double xd = md_.rb_inf_lb3d_(0) + mp_.resolution_ / 2; xd < md_.rb_inf_ub3d_(0); xd += mp_.resolution_)
      for (double yd = md_.rb_inf_lb3d_(1) + mp_.resolution_ / 2; yd < md_.rb_inf_ub3d_(1); yd += mp_.resolution_)
        for (double zd = lbz + mp_.resolution_ / 2; zd < ubz; zd += mp_.resolution_)
        {
          // Eigen::Vector3d relative_dir = (Eigen::Vector3d(xd, yd, zd) - md_.camera_pos_);
          // if (heading.dot(relative_dir.normalized()) > 0.5)
          // {
          int s = md_.occ_buf_inf_[globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)))];
          if (inf_sub && s > 0 && zd < mp_.visual_inflat_map_height_)
            cloud_inf.push_back(pcl::PointXYZ(xd, yd, zd));
          if (uk_sub && s == GRID_MAP_UNKNOWN_FLAG && zd < 1.0 && zd > 0.8) // TODO 只可视化0.8-1.0范围内的，不然可视化卡死
            cloud_uk.push_back(pcl::PointXYZ(xd, yd, zd));
          // }
        }

  sensor_msgs::PointCloud2 cloud_msg;
  if (inf_sub)
  {
    cloud_inf.width = cloud_inf.points.size();
    cloud_inf.height = 1;
    cloud_inf.is_dense = true;
    cloud_inf.header.frame_id = "world";
    pcl::toROSMsg(cloud_inf, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    map_inf_pub_.publish(cloud_msg);
  }
  if (uk_sub)
  {
    cloud_uk.width = cloud_uk.points.size();
    cloud_uk.height = 1;
    cloud_uk.is_dense = true;
    cloud_uk.header.frame_id = "world";
    pcl::toROSMsg(cloud_uk, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    map_unknown_pub_.publish(cloud_msg);
  }
}

void GridMap::publishESDF() const
{
  if (esdf_pub_.getNumSubscribers() <= 0)
    return;

  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = -1.0;
  const double max_dist = 5.0;
  const double esdf_slice_height = 1.0;

  for (int x = md_.rb_lb3i_(0); x <= md_.rb_ub3i_(0); ++x)
    for (int y = md_.rb_lb3i_(1); y <= md_.rb_ub3i_(1); ++y)
    {

      Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, y, md_.rb_lb3i_(2)));
      pos(2) = esdf_slice_height;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  // for (int x = md_.rb_lb3i_(0); x <= md_.rb_ub3i_(0); ++x)
  //   for (int z = md_.rb_lb3i_(2); z <= md_.rb_ub3i_(2); ++z)
  //   {

  //     Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, 0, z));

  //     dist = getDistance(pos);
  //     dist = min(dist, max_dist);
  //     dist = max(dist, min_dist);

  //     pt.x = pos(0);
  //     pt.y = pos(1);
  //     pt.z = pos(2);
  //     pt.intensity = (dist - min_dist) / (max_dist - min_dist);
  //     cloud.push_back(pt);
  //   }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  esdf_pub_.publish(cloud_msg);
}

void GridMap::cheakMapConsistancy(std::string where)
{
  ROS_ERROR("Not valid since I adopt hysteresis loop.");
  return;

  // for (int x = md_.rb_inf_lb3i_(0); x < md_.rb_inf_ub3i_(0); ++x)
  //   for (int y = md_.rb_inf_lb3i_(1); y < md_.rb_inf_ub3i_(1); ++y)
  //     for (int z = md_.rb_inf_lb3i_(2); z < md_.rb_inf_ub3i_(2); ++z)
  //     {
  //       int count = 0;
  //       for (int dx = -mp_.inf_grid_; dx <= mp_.inf_grid_; ++dx)
  //         for (int dy = -mp_.inf_grid_; dy <= mp_.inf_grid_; ++dy)
  //           for (int dz = -mp_.inf_grid_; dz <= mp_.inf_grid_; ++dz)
  //             if (isInBuf(Eigen::Vector3i(x + dx, y + dy, z + dz)) && occ_high(globalIdx2BufIdx(Eigen::Vector3i(x + dx, y + dy, z + dz))))
  //               count++;

  //       int infocc = md_.occ_buf_inf_[globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z))];
  //       bool occ = !isInBuf(Eigen::Vector3i(x, y, z)) ? 0 : occ_high(globalIdx2BufIdx(Eigen::Vector3i(x, y, z)));
  //       if (occ)
  //       {
  //         if (count != infocc - GRID_MAP_OBS_FLAG)
  //         {
  //           ROS_ERROR("%s: occ=%d, infocc=%d, count=%d, id=%d %d %d",
  //                     where.c_str(), occ, infocc, count, x, y, z);
  //         }
  //       }
  //       else
  //       {
  //         if (!((count == 0 && infocc == GRID_MAP_UNKNOWN_FLAG) || count == infocc))
  //         {
  //           ROS_ERROR("%s: occ=%d, infocc=%d, count=%d, id=%d %d %d",
  //                     where.c_str(), occ, infocc, count, x, y, z);
  //         }
  //       }
  //     }
}

void GridMap::testIndexingCost()
{
  if (!mp_.have_initialized_)
    return;

  double a = 0;
  unsigned long long b = 0;

  ros::Time t0 = ros::Time::now();
  int n21 = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = md_.rb_lb3i_(0); x <= md_.rb_ub3i_(0); ++x)
      for (int y = md_.rb_lb3i_(1); y <= md_.rb_ub3i_(1); ++y)
        for (int z = md_.rb_lb3i_(2); z <= md_.rb_ub3i_(2); ++z)
        {
          ++n21;
          b += z;
          b += x + y + z;
        }
  ros::Time t1 = ros::Time::now();

  n21 = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = md_.rb_lb3i_(0); x <= md_.rb_ub3i_(0); ++x)
      for (int y = md_.rb_lb3i_(1); y <= md_.rb_ub3i_(1); ++y)
        for (int z = md_.rb_lb3i_(2); z <= md_.rb_ub3i_(2); ++z)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(Eigen::Vector3i(x, y, z)); // 8396us = 7970
          ++n21;
          b += id_buf_inf_clr;
          b += x + y + z;
        }
  ros::Time t2 = ros::Time::now();

  int n32 = 0;
  for (int i = 0; i < 10; ++i)
    for (int x = md_.rb_lb3i_(0); x <= md_.rb_ub3i_(0); ++x)
      for (int y = md_.rb_lb3i_(1); y <= md_.rb_ub3i_(1); ++y)
        for (int z = md_.rb_lb3i_(2); z <= md_.rb_ub3i_(2); ++z)
        {
          Eigen::Vector3d pos = globalIdx2Pos(Eigen::Vector3i(x, y, z)); // 6553us = 6127
          ++n32;
          a += pos.sum();
          b += x + y + z;
        }
  ros::Time t3 = ros::Time::now();

  int n54 = 0;
  for (int i = 0; i < 10; ++i)
    for (double xd = md_.rb_lb3d_(0); xd <= md_.rb_ub3d_(0); xd += mp_.resolution_)
      for (double yd = md_.rb_lb3d_(1); yd <= md_.rb_ub3d_(1); yd += mp_.resolution_)
        for (double zd = md_.rb_lb3d_(2); zd <= md_.rb_ub3d_(2); zd += mp_.resolution_)
        {
          ++n54;
          a += xd + yd + zd;
          b += i + i + i;
        }
  ros::Time t4 = ros::Time::now();

  n54 = 0;
  for (int i = 0; i < 10; ++i)
    for (double xd = md_.rb_lb3d_(0); xd <= md_.rb_ub3d_(0); xd += mp_.resolution_)
      for (double yd = md_.rb_lb3d_(1); yd <= md_.rb_ub3d_(1); yd += mp_.resolution_)
        for (double zd = md_.rb_lb3d_(2); zd <= md_.rb_ub3d_(2); zd += mp_.resolution_)
        {
          Eigen::Vector3i idx = pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)); // 7088us = 478us
          ++n54;
          a += xd + yd + zd;
          b += idx.sum();
        }
  ros::Time t5 = ros::Time::now();

  int n65 = 0;
  for (int i = 0; i < 10; ++i)
    for (double xd = md_.rb_lb3d_(0); xd <= md_.rb_ub3d_(0); xd += mp_.resolution_)
      for (double yd = md_.rb_lb3d_(1); yd <= md_.rb_ub3d_(1); yd += mp_.resolution_)
        for (double zd = md_.rb_lb3d_(2); zd <= md_.rb_ub3d_(2); zd += mp_.resolution_)
        {
          int id_buf_inf_clr = globalIdx2InfBufIdx(pos2GlobalIdx(Eigen::Vector3d(xd, yd, zd)));
          ++n65;
          a += xd + yd + zd;
          b += id_buf_inf_clr;
        }
  ros::Time t6 = ros::Time::now();

  int n87 = 0;
  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occ_buf_.size(); ++i)
    {
      b += i;
      ++n87;
    }
  ros::Time t7 = ros::Time::now();

  n87 = 0;
  for (int i = 0; i < 10; ++i)
    for (size_t i = 0; i < md_.occ_buf_.size(); ++i)
    {
      Eigen::Vector3i idx = BufIdx2GlobalIdx(i); // 36939
      ++n87;
      b += i;
      b += idx.sum();
    }
  ros::Time t8 = ros::Time::now();

  int n = md_.occ_buf_.size() * 10;

  cout << "a=" << a << " b=" << b << endl;
  printf("iter=%d, t1-t0=%f, t2-t1=%f, t3-t2=%f, t4-t3=%f, t5-t4=%f, t6-t5=%f, t7-t6=%f, t8-t7=%f\n", n, (t1 - t0).toSec(), (t2 - t1).toSec(), (t3 - t2).toSec(), (t4 - t3).toSec(), (t5 - t4).toSec(), (t6 - t5).toSec(), (t7 - t6).toSec(), (t8 - t7).toSec());
  printf("globalIdx2InfBufIdx():%fns(1.88), globalIdx2Pos():%fns(0.70), pos2GlobalIdx():%fns(1.11), globalIdx2InfBufIdx(pos2GlobalIdx()):%fns(3.56), BufIdx2GlobalIdx():%fns(10.05)\n",
         ((t2 - t1) - (t1 - t0)).toSec() * 1e9 / n21, ((t3 - t2) - (t1 - t0)).toSec() * 1e9 / n32, ((t5 - t4) - (t4 - t3)).toSec() * 1e9 / n54, ((t6 - t5) - (t2 - t1)).toSec() * 1e9 / n65, ((t8 - t7) - (t7 - t6)).toSec() * 1e9 / n87);
}

/**************** class MapManager ****************/
void MapManager::initMapManager(ros::NodeHandle &nh)
{
  sml_.reset(new GridMap);
  sml_->initMap(nh);

  big_.reset(new GridMap);
  big_->initMap(nh);
  big_->paramAdjust(0.25, 20.0, 20.0, 5.0, true, false, true, "Big");

  map_use_ = SMALL;
  cur_ = sml_;
}

void MapManager::setMapUse(MAP_USE use)
{

  // map_use_ = use;

  switch (use)
  {
  case SMALL:
    map_use_ = use;
    cur_ = sml_;
    break;

  case LARGE:
    map_use_ = use;
    cur_ = big_;
    break;

  default:
    ROS_ERROR("Un-supported map use: %d", use);
    break;
  }
}
