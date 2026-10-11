#include "sensor_time.h"
#include <cassert>
#include <vector>

struct Point { int id; float curvature; };
int main()
{
  using namespace frontend_sensor_time;
  // Camera absent, late or far in the future must never block a ready scan.
  assert(next_cut(10.1, std::numeric_limits<double>::infinity(), true) == 10.1);
  assert(next_cut(10.1, 99999., true) == 10.1);
  assert(next_cut(10.1, 10.03, false) == 10.1);
  assert(next_cut(10.1, 10.03, true) == 10.03);
  assert(!ready_to_process(10.1, 10.2, .01, .2));
  assert(ready_to_process(10.1, 10.3, .01, .2));
  assert(ready_to_process(10.1, 10.1, .21, .2));
  std::vector<Point> packet, first, remaining, second, tail, third, empty;
  for (int i = 0; i < 100; ++i) packet.push_back({i, float(i)});
  partition(packet, 10., 10., 10.03, first, remaining);
  partition(remaining, 10.03, 10.03, 10.065, second, tail);
  partition(tail, 10.065, 10.065, 10.1, third, empty);
  assert(first.size() == 31 && second.size() == 35 && third.size() == 34 && empty.empty());
  std::vector<Point> restored;
  auto collect = [&](const std::vector<Point> &points, double origin)
  {
    for (const auto &p : points)
    {
      assert(p.id == int(restored.size()));
      assert(std::abs(origin + p.curvature * 0.001 - (10. + p.id * 0.001)) < 1e-7);
      restored.push_back(p);
    }
  };
  collect(first, 10.); collect(second, 10.03); collect(third, 10.065);
  assert(restored.size() == packet.size());
  // Epoch-sized stamps and a packet gap preserve each point's real timestamp.
  first.clear(); remaining.clear();
  partition(packet, 1726000000.2, 1726000000., 1726000000.15, first, remaining);
  assert(first.empty() && remaining.size() == 100);
  assert(std::abs(remaining.front().curvature - 50.) < .001);
}
