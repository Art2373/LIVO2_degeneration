#include "LIVMapper.h"
#include "sensor_time.h"
#include <algorithm>
#include <cmath>
#include <limits>

bool LIVMapper::syncMeasurementPackages(LidarMeasureGroup &meas)
{
  // This path never waits for a camera. A camera frame simply introduces an
  // additional chronological cut in the LiDAR stream when it is available.
  if (visual_pending_vio)
  {
    MeasureGroup m;
    m.lio_time = m.vio_time = meas.last_lio_update_time;
    m.img = visual_pending_image;
    meas.measures.clear();
    meas.measures.push_back(m);
    meas.lio_vio_flg = VIO;
    visual_pending_vio = false;
    visual_pending_image.release();
    noteDynamic("vio_dispatched", -1., m.vio_time, m.lio_time);
    return true;
  }
  if (imu_buffer.empty()) { noteDynamic("wait_imu", -1., -1., -1., -1, false); return false; }
  while (!lid_raw_data_buffer.empty() && lid_raw_data_buffer.front()->size() < 2)
  {
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
  }
  if (lid_raw_data_buffer.empty() && meas.pcl_proc_next->empty()) { noteDynamic("wait_lidar", -1., -1., -1., -1, false); return false; }
  if (meas.last_lio_update_time < 0.)
  {
    if (lid_header_time_buffer.empty()) return false;
    meas.last_lio_update_time = lid_header_time_buffer.front();
  }
  const double previous = meas.last_lio_update_time;
  while (!img_time_buffer.empty() && img_time_buffer.front() <= previous + 1e-6)
  {
    noteDynamic("image_drop_sync_past", img_time_buffer.front()-img_time_offset, img_time_buffer.front(), previous);
    img_time_buffer.pop_front();
    img_buffer.pop_front();
  }


  double cut = std::numeric_limits<double>::infinity();
  if (!meas.pcl_proc_next->empty())
    cut = previous + meas.pcl_proc_next->points.back().curvature * 0.001;
  if (!lid_raw_data_buffer.empty())
  {
    meas.lidar = lid_raw_data_buffer.front();
    const double raw_end = lid_header_time_buffer.front() + meas.lidar->points.back().curvature * 0.001;
    cut = std::min(cut, raw_end);
  }
  const double image_time = img_time_buffer.empty() ? std::numeric_limits<double>::infinity() : img_time_buffer.front();
  bool use_image = visual_processing_active && std::isfinite(image_time) && image_time <= cut;
  cut = frontend_sensor_time::next_cut(cut, image_time, visual_processing_active);
  if (!std::isfinite(cut) || cut <= previous + 1e-6)
  {
    noteDynamic("cut_nonadvancing", -1., image_time, cut);
    if (!lid_raw_data_buffer.empty()) { lid_raw_data_buffer.pop_front(); lid_header_time_buffer.pop_front(); }
    return false;
  }
  if (last_timestamp_imu < cut) { noteDynamic("wait_imu_coverage", -1., image_time, cut, -1, false); return false; }
  if (!frontend_sensor_time::ready_to_process(cut, sensor_latest_lidar_end,
      (ros::WallTime::now() - sensor_latest_lidar_arrival).toSec(), sensor_processing_delay)) { noteDynamic("wait_reorder", -1., image_time, cut, -1, false); return false; }

  PointCloudXYZI::Ptr current(new PointCloudXYZI()), next(new PointCloudXYZI());
  auto split = [&](const PointCloudXYZI &cloud, double origin)
  {
    frontend_sensor_time::partition(cloud, origin, previous, cut, *current, *next);
  };
  split(*meas.pcl_proc_next, previous);
  while (!lid_raw_data_buffer.empty() && lid_header_time_buffer.front() <= cut)
  {
    split(*lid_raw_data_buffer.front(), lid_header_time_buffer.front());
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
  }
  auto by_time = [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; };
  std::sort(current->begin(), current->end(), by_time);
  std::sort(next->begin(), next->end(), by_time);
  meas.pcl_proc_cur = current;
  meas.pcl_proc_next = next;
  meas.lidar_frame_beg_time = previous;
  meas.lidar_frame_end_time = cut;
  MeasureGroup m;
  m.lio_time = cut;
  while (!imu_buffer.empty() && imu_buffer.front()->header.stamp.toSec() <= cut)
  {
    if (imu_buffer.front()->header.stamp.toSec() >= previous) m.imu.push_back(imu_buffer.front());
    sensor_previous_imu = imu_buffer.front();
    imu_buffer.pop_front();
  }
  // Supply an exact endpoint using bracketing measurements. Keep the future
  // measurement queued for the next cut; very short image cuts remain valid.
  if (sensor_previous_imu && !imu_buffer.empty())
  {
    const auto &a = *sensor_previous_imu;
    const auto &b = *imu_buffer.front();
    const double ta = a.header.stamp.toSec(), tb = b.header.stamp.toSec();
    if (ta < cut && tb > ta && cut - ta < 0.05 && tb - cut < 0.05)
    {
      const double w = (cut - ta) / (tb - ta);
      sensor_msgs::Imu::Ptr end(new sensor_msgs::Imu(a));
      end->header.stamp.fromSec(cut);
      end->angular_velocity.x = (1-w)*a.angular_velocity.x + w*b.angular_velocity.x;
      end->angular_velocity.y = (1-w)*a.angular_velocity.y + w*b.angular_velocity.y;
      end->angular_velocity.z = (1-w)*a.angular_velocity.z + w*b.angular_velocity.z;
      end->linear_acceleration.x = (1-w)*a.linear_acceleration.x + w*b.linear_acceleration.x;
      end->linear_acceleration.y = (1-w)*a.linear_acceleration.y + w*b.linear_acceleration.y;
      end->linear_acceleration.z = (1-w)*a.linear_acceleration.z + w*b.linear_acceleration.z;
      m.imu.push_back(end);
    }
  }
  if (m.imu.empty())
  {
    noteDynamic("cut_without_imu", -1., image_time, cut, current->size());
    if (p_imu->imu_need_init) meas.last_lio_update_time = cut;
    else
    {
      // No state update happened, so retained points still need offsets from
      // the old state time. Drop this uncovered scan and wait for IMU coverage.
      for (auto &point : meas.pcl_proc_next->points)
        point.curvature += static_cast<float>((cut - previous) * 1000.);
      ROS_WARN_THROTTLE(2., "Missing IMU coverage; withholding this state update");
    }
    return false;
  }
  meas.measures.clear();
  meas.measures.push_back(m);
  meas.lio_vio_flg = LIO;
  if (use_image)
  {
    visual_pending_image = img_buffer.front();
    img_buffer.pop_front();
    img_time_buffer.pop_front();
    visual_pending_vio = !p_imu->imu_need_init && current->size() > 5;
    noteDynamic(visual_pending_vio ? "image_selected" : p_imu->imu_need_init ? "image_skip_imu_init" : "image_skip_lidar_points", image_time-img_time_offset, image_time, cut, current->size());
  }
  if (!use_image) noteDynamic("cut_lidar_only", -1., std::isfinite(image_time) ? image_time : -1., cut, current->size());
  lidar_pushed = false;
  return true;
}
