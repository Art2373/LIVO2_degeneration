#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace frontend_sensor_time
{
// Point.curvature holds milliseconds. Works with both PCL and std::vector.
// A point belongs to exactly one side; future offsets are rebased to the cut.
template <class Cloud>
void partition(const Cloud &input, double origin, double previous, double cut,
               Cloud &current, Cloud &future)
{
  for (const auto &source : input)
  {
    const double stamp = origin + source.curvature * 0.001;
    if (!std::isfinite(stamp) || stamp < previous - 1e-6) continue;
    auto point = source;
    if (stamp <= cut + 1e-9)
    {
      point.curvature = static_cast<float>(std::max(0., stamp - previous) * 1000.);
      current.push_back(point);
    }
    else
    {
      point.curvature = static_cast<float>((stamp - cut) * 1000.);
      future.push_back(point);
    }
  }
}

inline double next_cut(double scan_end, double image_time, bool camera_ready)
{
  return camera_ready && std::isfinite(image_time) ? std::min(scan_end, image_time) : scan_end;
}

// Bounded reordering lag, independent of camera presence. Flush the final scan
// after the wall-time deadline even if no further sensor packet arrives.
inline bool ready_to_process(double cut, double latest_end, double since_arrival, double lag)
{
  return cut <= latest_end - lag + 1e-6 || since_arrival >= lag;
}
}
