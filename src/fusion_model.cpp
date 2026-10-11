#include "LIVMapper.h"
#include <iomanip>
#include <cmath>
#include <sstream>

void LIVMapper::fusionModelCallback(const livo2_degeneration::FusionModel::ConstPtr &msg)
{
  // Only the explicit given-parameter provider is accepted on this interface.
  if (!fusion_model_en || msg->source != "dataset_given_reference") return;
  fusion_model_result = msg;
  model_last_heartbeat = ros::WallTime::now();
  traceDynamic("heartbeat");
}

void LIVMapper::updateFusionModel()
{
  const bool fresh = fusion_model_result &&
      (ros::WallTime::now() - model_last_heartbeat).toSec() < model_heartbeat_timeout;
  dynamic_service_alive = fresh;
  if (!fresh) {
    dynamic_model_usable = false;
    if (visual_policy_config.enabled) updateVisualLifecycle();
    else withdrawDynamicCamera("heartbeat_timeout");
    return;
  }
  const auto &msg = *fusion_model_result;
  if (!visual_policy_config.enabled && visual_processing_active && msg.revision == model_applied_revision) return;
  const auto &q = msg.imu_from_camera.rotation;
  const auto &t = msg.imu_from_camera.translation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  const V3D translation(t.x, t.y, t.z);
  const auto &info = msg.camera_info;
  bool model_ok = info.width >= 32 && info.height >= 32 && !info.header.frame_id.empty() &&
      info.K[0] > 0 && info.K[4] > 0 &&
      (info.distortion_model == "plumb_bob" || info.distortion_model.empty()) &&
      (info.D.empty() || info.D.size() == 4 || info.D.size() == 5);
  for (double v : info.K) model_ok &= std::isfinite(v);
  for (double v : info.D) model_ok &= std::isfinite(v);
  if (!model_ok || !translation.allFinite() || !std::isfinite(rotation.norm()) ||
      std::abs(rotation.norm() - 1.) > .01 || translation.norm() > model_max_baseline ||
      !std::isfinite(msg.camera_time_offset) || msg.reference_sha256.size() != 64)
  {
    dynamic_model_usable = false;
    if (visual_policy_config.enabled) updateVisualLifecycle();
    else withdrawDynamicCamera("malformed_given_reference");
    ROS_WARN_THROTTLE(2., "Rejected malformed given reference camera model");
    return;
  }
  if (visual_policy_config.enabled) {
    if (msg.session_id.empty()) {
      dynamic_model_usable = false;
      updateVisualLifecycle();
      ROS_WARN_THROTTLE(2., "Managed given-reference mode requires an explicit publisher session");
      return;
    }
    stageDynamicModel(rotation, translation, info, msg.camera_time_offset, msg.revision,
        "given:" + msg.session_id + ":" + msg.reference_sha256);
    updateVisualLifecycle();
    return;
  }
  dynamic_model_usable = true;
  // Do not manufacture geometric quality scores. This is a trusted supplied model.
  if (p_imu->imu_need_init || !lidar_map_inited || dynamic_lio_effective == 0 ||
      dynamic_last_image_stamp < 0 || dynamic_last_image_width != int(info.width) ||
      dynamic_last_image_height != int(info.height) || dynamic_last_image_frame != info.header.frame_id ||
      visual_pending_vio) return;
  applyDynamicCamera(rotation, translation, info, msg.camera_time_offset, msg.revision);
  ROS_INFO("Dataset-given camera reference applied: revision %u, offset %.6f s (given fusion parameters)",
      msg.revision, msg.camera_time_offset);
}

void LIVMapper::withdrawDynamicCamera(const char *reason)
{
  const bool was_active = visual_processing_active;
  if (was_active) {
    traceDynamic(reason);
    ROS_WARN("Visual processing withdrawn; continuing LiDAR/IMU mapping");
    ++dynamic_cache_clears;
    vio_manager->resetVisualMap();
    pcl_wait_pub->clear(); // Color accumulation belongs to the invalid visual session.
  }
  visual_processing_active = false;
  visual_pending_vio = false;
  visual_pending_image.release();
  img_buffer.clear();
  img_time_buffer.clear();
  slam_mode_ = ONLY_LIO;
  if (was_active) traceDynamic("withdrawn");
}

void LIVMapper::traceDynamic(const char *event)
{
  if (!dynamic_trace.is_open()) return;
  const auto q = Eigen::Quaterniond(_state.rot_end);
  dynamic_trace << std::setprecision(17) << "{\"event\":\"" << event
    << "\",\"wall\":" << ros::WallTime::now().toSec()
    << ",\"clock\":" << ros::Time::now().toSec()
    << ",\"state_time\":" << LidarMeasures.last_lio_update_time
    << ",\"active\":" << visual_processing_active << ",\"revision\":" << model_applied_revision
    << ",\"source\":\"" << "dataset_given_reference"
    << "\",\"heartbeat_wall\":" << model_last_heartbeat.toSec()
    << ",\"offset\":" << img_time_offset << ",\"imu_initializing\":" << p_imu->imu_need_init
    << ",\"lio_calls\":" << dynamic_lio_calls << ",\"lio_effective\":" << dynamic_lio_effective
    << ",\"vio_calls\":" << dynamic_vio_calls << ",\"vio_effective\":" << dynamic_vio_effective
    << ",\"lio_steps\":" << voxelmap_manager->diag_effective_updates
    << ",\"lio_nonfinite\":" << voxelmap_manager->diag_nonfinite
    << ",\"vio_steps\":" << vio_manager->diag_accepted << ",\"vio_reverted\":" << vio_manager->diag_reverted
    << ",\"vio_pixels\":" << vio_manager->diag_pixels << ",\"vio_nonfinite\":" << vio_manager->diag_nonfinite
    << ",\"vio_mse\":" << vio_manager->diag_last_mse
    << ",\"visual_observations\":" << (vio_manager->visual_submap ? vio_manager->visual_submap->voxel_points.size() : 0)
    << ",\"visual_voxels\":" << vio_manager->feat_map.size()
    << ",\"warp_cache\":" << vio_manager->warp_map.size()
    << ",\"reference_frame\":" << bool(vio_manager->new_frame_)
    << ",\"rgb_cached\":" << !vio_manager->img_rgb.empty()
    << ",\"color_cache\":" << pcl_wait_pub->size()
    << ",\"pending_vio\":" << visual_pending_vio
    << ",\"pending_image\":" << !visual_pending_image.empty()
    << ",\"image_queue\":" << img_buffer.size()
    << ",\"raw_queue\":" << lid_raw_data_buffer.size() << ",\"imu_queue\":" << imu_buffer.size()
    << ",\"received_images\":" << dynamic_images_received << ",\"inactive_images\":" << dynamic_images_inactive
    << ",\"queued_images\":" << dynamic_images_queued
    << ",\"last_raw_image\":" << dynamic_last_image_stamp
    << ",\"last_corrected_image\":" << last_timestamp_img
    << ",\"queue_first_stamp\":" << (img_time_buffer.empty() ? -1. : img_time_buffer.front())
    << ",\"queue_last_stamp\":" << (img_time_buffer.empty() ? -1. : img_time_buffer.back())
    << ",\"latest_lidar_end\":" << sensor_latest_lidar_end
    << ",\"latest_imu\":" << last_timestamp_imu
    << ",\"latest_angular_speed\":" << visual_latest_angular_speed
    << ",\"management_enabled\":" << visual_policy_config.enabled
    << ",\"health_state\":\"" << (visual_policy_config.enabled ? visual_lifecycle::stateName(visual_health.state) : "LEGACY_PERMISSION_ONLY") << "\""
    << ",\"health_previous\":\"" << visual_lifecycle::stateName(visual_health.previous) << "\""
    << ",\"policy_reason\":\"" << visual_health.reason << "\""
    << ",\"service_alive\":" << dynamic_service_alive
    << ",\"model_usable\":" << dynamic_model_usable
    << ",\"input_fresh\":" << visual_health.image_fresh
    << ",\"allow_visual_processing\":" << visual_processing_active
    << ",\"probe_changes_filter\":" << (visual_policy_config.enabled && visual_health.probing_changes_filter)
    << ",\"model_epoch\":" << dynamic_model_epoch
    << ",\"session_epoch\":" << dynamic_model_epoch
    << ",\"probe_attempt\":" << visual_health.probe_attempt
    << ",\"probe_calls\":" << visual_health.probe_calls
    << ",\"recovery_schedule\":\"" << visual_lifecycle::recoveryScheduleName(visual_policy_config.recovery_schedule) << "\""
    << ",\"probe_started_sensor\":" << (std::isfinite(visual_health.probe_started_sensor) ? visual_health.probe_started_sensor : -1.)
    << ",\"cooldown_until_sensor\":" << (std::isfinite(visual_health.cooldown_until) ? visual_health.cooldown_until : -1.)
    << ",\"consecutive_successes\":" << visual_health.consecutive_successes
    << ",\"consecutive_failures\":" << visual_health.consecutive_failures
    << ",\"actual_update_time\":" << (visual_health.has_retained_update ? visual_health.last_retained_update_sensor : -1.)
    << ",\"last_decoded_legal_image_wall\":" << (std::isfinite(dynamic_legal_image_wall) ? dynamic_legal_image_wall : -1.)
    << ",\"last_decoded_legal_image_raw\":" << dynamic_legal_image_raw
    << ",\"last_lidar_arrival_wall\":" << sensor_latest_lidar_arrival.toSec()
    << ",\"last_imu_arrival_wall\":" << dynamic_imu_wall
    << ",\"cache_clear_count\":" << dynamic_cache_clears
    << ",\"vio_retained_finite\":" << dynamic_vio_retained_finite
    << ",\"feedback_schema\":1,\"feedback_mode\":\"" << visual_feedback::modeName(visual_policy_config.feedback_mode) << "\""
    << ",\"feedback_call_id\":" << dynamic_feedback_event.call_id
    << ",\"feedback_model_epoch\":" << dynamic_feedback_event.model_epoch
    << ",\"feedback_sensor_stamp\":" << (std::isfinite(dynamic_feedback_event.sensor_stamp) ? dynamic_feedback_event.sensor_stamp : -1.)
    << ",\"feedback_executed\":" << dynamic_feedback_event.result.executed
    << ",\"feedback_completed\":" << dynamic_feedback_event.result.completed
    << ",\"feedback_linearizations\":" << dynamic_feedback_event.result.attempted_linearizations
    << ",\"feedback_measurement_rows\":" << dynamic_feedback_event.result.last_measurement_rows
    << ",\"feedback_last_measurement_finite\":" << dynamic_feedback_event.result.last_measurement_finite
    << ",\"feedback_last_pose_information\":" << dynamic_feedback_event.result.last_pose_information
    << ",\"feedback_last_exposure_information\":" << dynamic_feedback_event.result.last_exposure_information
    << ",\"feedback_surviving_accepts\":" << dynamic_feedback_event.result.path.surviving_accepts
    << ",\"feedback_surviving_pose_information\":" << dynamic_feedback_event.result.path.pose_information
    << ",\"feedback_surviving_exposure_information\":" << dynamic_feedback_event.result.path.exposure_information
    << ",\"feedback_last_surviving_step\":" << dynamic_feedback_event.result.path.last_step
    << ",\"feedback_covariance_basis_step\":" << dynamic_feedback_event.result.covariance_basis_step
    << ",\"feedback_covariance_basis_survives\":" << dynamic_feedback_event.result.covariance_basis_survives
    << ",\"feedback_covariance_committed\":" << dynamic_feedback_event.result.covariance_committed
    << ",\"feedback_covariance_finite\":" << dynamic_feedback_event.result.covariance_finite
    << ",\"feedback_nonfinite_attempt\":" << dynamic_feedback_event.result.nonfinite_attempt
    << ",\"feedback_final_finite\":" << dynamic_feedback_event.result.final_finite
    << ",\"feedback_eligible\":" << dynamic_feedback_eligible
    << ",\"feedback_eligible_calls\":" << dynamic_feedback_eligible_calls
    << ",\"feedback_eligibility_reason\":\"" << dynamic_feedback_event.result.reason(dynamic_feedback_event.legacy_retained) << "\""
    << ",\"feedback_last_eligible_image_time\":" << dynamic_feedback_last_eligible_image
    << ",\"legacy_last_retained_image_time\":" << dynamic_legacy_last_retained_image
    << ",\"policy_result_counted\":" << dynamic_feedback_counted
    << ",\"policy_result_success\":" << dynamic_feedback_success
    << ",\"policy_event_ignored\":" << visual_health.event_ignored
    << ",\"policy_event_reason\":\"" << visual_health.event_reason << "\""
    << ",\"policy_last_success_time\":" << (visual_health.has_retained_update ? visual_health.last_retained_update_sensor : -1.)
    << ",\"vio_covariance_max_delta\":" << (std::isfinite(dynamic_vio_cov_max_delta) ? dynamic_vio_cov_max_delta : -1.)
    << ",\"state_finite\":" << finiteVisualState()
    << ",\"vio_image_corrected_time\":" << dynamic_vio_image_time
    << ",\"vio_image_raw_time\":" << (dynamic_vio_image_time < 0. ? -1. : dynamic_vio_image_time - img_time_offset)
    << ",\"event_raw\":" << dynamic_event_raw
    << ",\"event_corrected\":" << dynamic_event_corrected
    << ",\"event_cut\":" << (std::isfinite(dynamic_event_cut) ? dynamic_event_cut : -1.)
    << ",\"event_points\":" << dynamic_event_points
    << ",\"last_reason\":\"" << dynamic_event_reason << "\""
    << ",\"vio_translation_increment\":" << dynamic_vio_delta_position
    << ",\"vio_rotation_increment_rad\":" << dynamic_vio_delta_angle
    << ",\"position\":[" << _state.pos_end.x() << ',' << _state.pos_end.y() << ',' << _state.pos_end.z()
    << "],\"quaternion_xyzw\":[" << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
  if (std::string(event) == "applied") {
    dynamic_trace << ",\"R_CL\":[";
    for (size_t i=0; i<cameraextrinR.size(); ++i) dynamic_trace << (i ? "," : "") << cameraextrinR[i];
    dynamic_trace << "],\"t_CL\":[";
    for (size_t i=0; i<cameraextrinT.size(); ++i) dynamic_trace << (i ? "," : "") << cameraextrinT[i];
    dynamic_trace << "],\"camera_width\":" << model_camera_width << ",\"camera_height\":" << model_camera_height
      << ",\"image_scale\":" << model_image_scale << ",\"scaled_fx\":" << vio_manager->fx
      << ",\"scaled_fy\":" << vio_manager->fy;
  }
  dynamic_trace << ",\"reason_counts\":{";
  bool first = true;
  for (const auto &item : dynamic_reasons) {
    dynamic_trace << (first ? "" : ",") << "\"" << item.first << "\":" << item.second;
    first = false;
  }
  dynamic_trace << "}}\n";
  dynamic_trace.flush();
}

void LIVMapper::noteDynamic(const char *reason, double raw, double corrected, double cut, long points, bool emit)
{
  ++dynamic_reasons[reason];
  if (!dynamic_schedule_trace) return;
  dynamic_event_reason = reason;
  dynamic_event_raw = std::isfinite(raw) ? raw : -1.;
  dynamic_event_corrected = std::isfinite(corrected) ? corrected : -1.;
  dynamic_event_cut = cut;
  dynamic_event_points = points;
  if (emit) traceDynamic(reason);
}

void LIVMapper::stageDynamicModel(const Eigen::Quaterniond &rotation, const V3D &translation,
    const sensor_msgs::CameraInfo &info, double offset, uint32_t revision, const std::string &source_key)
{
  // Headers change every frame; only the actual model, revision and source
  // session form the model identity. All callbacks and mutations use spinOnce.
  std::ostringstream key;
  key << std::setprecision(17) << source_key << ':' << revision << ':' << offset
      << ':' << rotation.coeffs().transpose() << ':' << translation.transpose()
      << ':' << info.width << ':' << info.height << ':' << info.header.frame_id << ':' << info.distortion_model;
  for (double v : info.K) key << ':' << v;
  for (double v : info.D) key << ':' << v;
  for (double v : info.R) key << ':' << v;
  for (double v : info.P) key << ':' << v;
  if (!dynamic_model_usable || key.str() != dynamic_model_key) {
    ++dynamic_model_epoch;
    dynamic_model_key = key.str();
    // Cached CameraInfo and pre-session image metadata cannot establish input
    // freshness. At least one new, decoded, monotonic image must arrive.
    dynamic_legal_image_wall = visual_lifecycle::noTime();
    dynamic_model_rotation = rotation;
    dynamic_model_translation = translation;
    dynamic_model_info = info;
    dynamic_model_offset = offset;
    dynamic_model_revision = revision;
    noteDynamic("model_epoch_staged");
  }
  dynamic_model_usable = true;
}

visual_lifecycle::Context LIVMapper::visualContext() const
{
  visual_lifecycle::Context context;
  context.wall_now = ros::WallTime::now().toSec();
  context.sensor_now = LidarMeasures.last_lio_update_time;
  context.last_legal_image_wall = dynamic_legal_image_wall;
  context.service_fresh = dynamic_service_alive &&
      context.wall_now - model_last_heartbeat.toSec() < model_heartbeat_timeout;
  context.model_usable = dynamic_model_usable;
  context.lio_ready = !p_imu->imu_need_init && lidar_map_inited && dynamic_lio_effective > 0;
  context.model_epoch = dynamic_model_epoch;
  return context;
}

void LIVMapper::updateVisualLifecycle()
{
  applyVisualDecision(visual_policy.poll(visualContext()));
}

void LIVMapper::applyVisualDecision(const visual_lifecycle::Decision &decision)
{
  visual_health = decision;
  if (decision.clear_visual_cache) withdrawDynamicCamera(decision.reason);
  if (decision.allow_visual_processing && !visual_processing_active && !visual_pending_vio) {
    applyDynamicCamera(dynamic_model_rotation, dynamic_model_translation, dynamic_model_info,
        dynamic_model_offset, dynamic_model_revision);
    traceDynamic("probe_started_real_filter");
  }
  if (decision.changed || decision.clear_visual_cache) traceDynamic("health_transition");
}

bool LIVMapper::finiteVisualState() const
{
  return _state.rot_end.allFinite() && _state.pos_end.allFinite() && _state.vel_end.allFinite()
      && _state.bias_g.allFinite() && _state.bias_a.allFinite() && _state.gravity.allFinite()
      && std::isfinite(_state.inv_expo_time) && _state.cov.allFinite();
}

void LIVMapper::applyDynamicCamera(const Eigen::Quaterniond &rotation, const V3D &translation,
    const sensor_msgs::CameraInfo &info, double offset, uint32_t revision)
{
  // Apply on the mapper thread, never in a model callback or midway
  // through an LIO/VIO measurement pair. Old visual patches are invalidated.
  if (visual_pending_vio) return;
  ++dynamic_cache_clears;
  vio_manager->resetVisualMap();
  delete vio_manager->cam;
  vio_manager->cam = nullptr;
  model_camera_width = info.width;
  model_camera_height = info.height;
  model_camera_frame = info.header.frame_id;
  const cv::Mat K = (cv::Mat_<double>(3, 3) << info.K[0], info.K[1], info.K[2],
      info.K[3], info.K[4], info.K[5], info.K[6], info.K[7], info.K[8]);
  cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
  for (size_t i = 0; i < info.D.size(); ++i) D.at<double>(0, i) = info.D[i];
  cv::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F), K,
      cv::Size(info.width, info.height), CV_32FC1, model_rectify_x, model_rectify_y);
  vio_manager->cam = new vk::PinholeCamera(info.width, info.height, model_image_scale,
      info.K[0], info.K[4], info.K[2], info.K[5]);

  // T_CL = inverse(T_IC) * T_IL. CameraInfo refers to the COLOR OPTICAL frame.
  const M3D Rci = rotation.normalized().toRotationMatrix().transpose();
  const M3D Rcl = Rci * extR;
  const V3D Pcl = Rci * (extT - translation);
  cameraextrinR.resize(9);
  cameraextrinT.resize(3);
  for (int row = 0; row < 3; ++row)
  {
    cameraextrinT[row] = Pcl[row];
    for (int col = 0; col < 3; ++col) cameraextrinR[3 * row + col] = Rcl(row, col);
  }
  configureVisualModule(); // Refreshes Rci/Pci and the cached visual Jacobians.
  img_time_offset = offset;
  exposure_time_init = 0.; // Do not apply a second, unrelated timestamp shift.
  img_buffer.clear();
  img_time_buffer.clear();
  pcl_wait_pub->clear();
  last_timestamp_img = -1.;
  model_applied_revision = revision;
  visual_processing_active = true;
  slam_mode_ = LIVO;
  traceDynamic("applied");
}

