// Standalone: g++ -std=c++14 -Iinclude tests/test_image_retention.cpp -o /tmp/test_image_retention
// Tests timestamp scheduling only, not visual geometry, SLAM, or real run success.
#include "image_retention.h"
#include "sensor_time.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <set>
#include <vector>

namespace
{
struct Result
{
  std::vector<double> selected, selected_at, discarded, cuts;
  std::deque<double> remaining;
  std::size_t max_size = 0;
  double state = 0;
};

// Fixed counterexample: 30 Hz image stamps (.01 + k/30) with +.1 s image
// compensation and .05 s arrival delay; 10 Hz LiDAR and .2 s reorder lag.
// Sensor time progresses at half wall speed, as in the archived experiments.
// Images stop after 90 inputs; existing LiDAR/IMU continues through t=5. No
// observations are added for EOF drain; only the original wall wait is advanced.
Result simulate(bool protect_head, bool have_images = true)
{
  std::map<long long, double> arrivals;
  std::set<long long> schedule;
  if (have_images)
    for (int j = 0; j < 90; ++j)
    {
      const long long when = std::llround((j / 30. + .06) * 1e6);
      arrivals[when] = j / 30. + .11;
      schedule.insert(when);
    }
  for (long long t = 0; t <= 6000000; t += 1000) schedule.insert(t);
  std::deque<int> payload;
  std::deque<double> stamps, raw_ends;
  double latest_lidar = 0., latest_imu = 0., last_lidar_wall = 0.;
  Result result;
  for (long long usec : schedule)
  {
    const double sensor = usec / 1e6, wall = sensor * 2.;
    if (usec > 0 && usec <= 5000000 && usec % 100000 == 0)
    {
      raw_ends.push_back(sensor);
      latest_lidar = latest_imu = sensor;
      last_lidar_wall = wall;
    }
    auto incoming = arrivals.find(usec);
    if (incoming != arrivals.end())
    {
      assert(incoming->second > result.state);
      payload.push_back(static_cast<int>(usec));
      stamps.push_back(incoming->second);
      if (protect_head)
        frontend_image_queue::trim_keep_head(payload, stamps, 8,
          [&](std::size_t, double stamp) { result.discarded.push_back(stamp); });
      else
        while (stamps.size() > 8)
        {
          result.discarded.push_back(stamps.front());
          stamps.pop_front(); payload.pop_front();
        }
      result.max_size = std::max(result.max_size, stamps.size());
      assert(stamps.size() <= 8 && payload.size() == stamps.size());
      assert(std::is_sorted(stamps.begin(), stamps.end()));
    }
    while (!raw_ends.empty())
    {
      while (!stamps.empty() && stamps.front() <= result.state + 1e-6)
      {
        result.discarded.push_back(stamps.front());
        stamps.pop_front(); payload.pop_front();
      }
      const double image = stamps.empty() ? INFINITY : stamps.front();
      const double cut = frontend_sensor_time::next_cut(raw_ends.front(), image, true);
      if (cut > latest_imu || !frontend_sensor_time::ready_to_process(
            cut, latest_lidar, wall - last_lidar_wall, .2)) break;
      assert(cut > result.state + 1e-6);
      if (image <= cut)
      {
        result.selected.push_back(image); result.selected_at.push_back(sensor);
        stamps.pop_front(); payload.pop_front();
      }
      result.state = cut; result.cuts.push_back(cut);
      while (!raw_ends.empty() && raw_ends.front() <= cut + 1e-9) raw_ends.pop_front();
    }
  }
  result.remaining = stamps;
  std::set<double> identities(result.selected.begin(), result.selected.end());
  assert(identities.size() == result.selected.size());
  assert(std::is_sorted(result.selected.begin(), result.selected.end()));
  assert(std::is_sorted(result.cuts.begin(), result.cuts.end()));
  for (double dropped : result.discarded) assert(!identities.count(dropped));
  assert(result.selected.size() + result.discarded.size() + result.remaining.size()
         == (have_images ? 90u : 0u));
  return result;
}
}

int main()
{
  using namespace frontend_image_queue;
  assert(overflow_drop_index(8, 8) == 8);
  assert(overflow_drop_index(9, 8) == 1);
  assert(overflow_drop_index(2, 1) == 1);
  bool rejected = false;
  try { overflow_drop_index(1, 0); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
  std::deque<int> images{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::deque<double> times{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  std::vector<double> dropped;
  assert(trim_keep_head(images, times, 8,
      [&](std::size_t index, double time) { assert(index == 1); dropped.push_back(time); }) == 2);
  assert((dropped == std::vector<double>{1., 2.}));
  assert(images.front() == 0 && images.back() == 9 && images[1] == 3);
  for (std::size_t i = 0; i < times.size(); ++i) assert(images[i] == times[i]);
  auto old = simulate(false), revised = simulate(true);
  auto count_middle = [](const Result &r)
  {
    return std::count_if(r.selected_at.begin(), r.selected_at.end(),
                        [](double t) { return t > .5 && t < 2.9; });
  };
  assert(count_middle(old) == 0);
  assert(count_middle(revised) > 0);
  assert(old.max_size == 8 && revised.max_size == 8);
  assert(old.remaining.empty() && revised.remaining.empty());
  assert(old.state == 5. && revised.state == 5.);
  auto no_old = simulate(false, false), no_new = simulate(true, false);
  assert(no_old.cuts == no_new.cuts && no_old.state == no_new.state);
  // Wall readiness does not create IMU coverage. Future sensor support remains
  // a separate precondition; this timestamp cannot be consumed at EOF.
  const double unsupported_image = 1.2, latest_imu = 1.;
  assert(frontend_sensor_time::ready_to_process(unsupported_image, 1., 10., .2));
  assert(!(unsupported_image <= latest_imu));
  // Retention alone does not reject late images: the existing callback/sync owns
  // this check. The policy preserves the head; it never rewrites its identity.
  std::cout << "PASS retention pairing/bounds/order/unique identity; old starvation="
            << count_middle(old) << ", protected-head selections=" << count_middle(revised)
            << "; total selected old/new=" << old.selected.size() << "/"
            << revised.selected.size() << "; existing-supported EOF drain and no-camera invariant\n";
}
