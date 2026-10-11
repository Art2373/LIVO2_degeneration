/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <livo2_degeneration/FusionModel.h>
#include <sensor_msgs/CameraInfo.h>
#include "visual_lifecycle.h"

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const LivoxMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
  void configureVisualModule();
  void fusionModelCallback(const livo2_degeneration::FusionModel::ConstPtr &msg);
  void updateFusionModel();
  void applyDynamicCamera(const Eigen::Quaterniond &rotation, const V3D &translation,
      const sensor_msgs::CameraInfo &info, double offset, uint32_t revision);
  void withdrawDynamicCamera(const char *reason);
  void traceDynamic(const char *event);
  void noteDynamic(const char *reason, double raw = -1., double corrected = -1., double cut = -1., long points = -1, bool emit = true);
  bool dynamic_schedule_trace = false;
  visual_lifecycle::Config visual_policy_config;
  visual_lifecycle::Manager visual_policy;
  visual_lifecycle::Decision visual_health;
  bool dynamic_service_alive = false, dynamic_model_usable = false;
  uint64_t dynamic_model_epoch = 0, dynamic_cache_clears = 0;
  std::string dynamic_model_key;
  Eigen::Quaterniond dynamic_model_rotation = Eigen::Quaterniond::Identity();
  V3D dynamic_model_translation = V3D::Zero();
  sensor_msgs::CameraInfo dynamic_model_info;
  double dynamic_model_offset = 0.;
  uint32_t dynamic_model_revision = 0;
  double dynamic_legal_image_wall = visual_lifecycle::noTime(), dynamic_legal_image_raw = -1.;
  double dynamic_imu_wall = -1.;
  double dynamic_vio_image_time = -1.;
  bool dynamic_vio_retained_finite = false;
  visual_lifecycle::Event dynamic_feedback_event;
  bool dynamic_feedback_eligible = false;
  bool dynamic_feedback_counted = false, dynamic_feedback_success = false;
  uint64_t dynamic_feedback_eligible_calls = 0;
  double dynamic_legacy_last_retained_image = -1.;
  double dynamic_feedback_last_eligible_image = -1.;
  double dynamic_vio_cov_max_delta = 0.;
  bool finiteVisualState() const;
  void stageDynamicModel(const Eigen::Quaterniond &rotation, const V3D &translation,
      const sensor_msgs::CameraInfo &info, double offset, uint32_t revision, const std::string &source_key);
  visual_lifecycle::Context visualContext() const;
  void updateVisualLifecycle();
  void applyVisualDecision(const visual_lifecycle::Decision &decision);
  std::unordered_map<std::string, uint64_t> dynamic_reasons;
  double dynamic_event_raw = -1., dynamic_event_corrected = -1., dynamic_event_cut = -1.;
  long dynamic_event_points = -1;
  std::string dynamic_event_reason;
  bool fusion_model_en = true, dynamic_visual_en = true;
  livo2_degeneration::FusionModel::ConstPtr fusion_model_result;
  std::ofstream dynamic_trace;
  ros::WallTime dynamic_trace_last;
  uint64_t dynamic_lio_calls = 0, dynamic_lio_effective = 0;
  uint64_t dynamic_vio_calls = 0, dynamic_vio_effective = 0;
  uint64_t dynamic_images_received = 0, dynamic_images_inactive = 0, dynamic_images_queued = 0;
  double dynamic_last_image_stamp = -1, dynamic_vio_delta_position = 0, dynamic_vio_delta_angle = 0;
  int dynamic_last_image_width = 0, dynamic_last_image_height = 0;
  std::string dynamic_last_image_frame;
  bool syncMeasurementPackages(LidarMeasureGroup &meas);

  bool visual_ready = false, visual_processing_active = false;
  bool visual_pending_vio = false;
  double model_image_scale = 0.5, model_heartbeat_timeout = 3.0;
  double model_max_baseline = 1.0;
  double visual_max_angular_speed = 0.8, visual_latest_angular_speed = 0.;
  double sensor_processing_delay = 0.2, sensor_latest_lidar_end = -1.;
  uint32_t model_applied_revision = 0;
  ros::WallTime model_last_heartbeat;
  ros::WallTime sensor_latest_lidar_arrival;
  ros::Subscriber sub_fusion_model;
  ros::Publisher pubVisualColored;
  cv::Mat visual_pending_image, model_rectify_x, model_rectify_y;
  sensor_msgs::Imu::ConstPtr sensor_previous_imu;
  int model_camera_width = 0, model_camera_height = 0;
  std::string model_camera_frame;

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, img_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int img_save_interval = 1, pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width = 0, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_visual_pos, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif
