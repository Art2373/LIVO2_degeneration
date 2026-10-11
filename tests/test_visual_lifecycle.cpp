// Pure policy tests: no ROS, real images, hardware, or manufactured filter state.
#include "visual_lifecycle.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

using namespace visual_lifecycle;

// Test-only adapter creates explicit execution/measurement/commit evidence.
// It does not set a production state or bypass the real manager transitions.
class TestManager : public visual_lifecycle::Manager {
public:
  explicit TestManager(Config config=Config()) : visual_lifecycle::Manager(config) {}
  Decision onVioResult(const Context &c, double stamp, bool informative) {
    Event e; e.call_id=++counter_; e.model_epoch=c.model_epoch;
    e.sensor_stamp=stamp; e.executed=true; e.legacy_retained=informative;
    e.result.executed=e.result.completed=e.result.final_finite=true;
    e.result.path.surviving_accepts=informative?1:0;
    e.result.path.pose_information=informative;
    e.result.covariance_committed=e.result.covariance_finite=informative;
    return visual_lifecycle::Manager::onVioResult(c,e);
  }
private:
  std::uint64_t counter_=0;
};


static Config enabled()
{
  Config cfg;
  cfg.enabled = true;
  return cfg;
}

static Context healthy(double sensor = 10., double wall = 1000.)
{
  Context c;
  c.wall_now = wall;
  c.sensor_now = sensor;
  c.last_legal_image_wall = wall;
  c.service_fresh = c.model_usable = c.lio_ready = true;
  c.model_epoch = 1;
  return c;
}

static bool isReason(const Decision &d, const char *reason)
{
  return std::strcmp(d.reason, reason) == 0;
}

static Decision fuse(TestManager &manager, double begin = 10.)
{
  Decision d = manager.poll(healthy(begin));
  assert(d.state == State::PROBING);
  for (double delta : {0., .125, .25})
    d = manager.onVioResult(healthy(begin + delta), begin + delta, true);
  assert(d.state == State::FUSING);
  return d;
}

static void explicitly_disabled()
{
  Config config; config.enabled=false; TestManager m(config);
  auto d = m.poll(healthy());
  assert(d.state == State::OFF && isReason(d, "disabled"));
  assert(!d.allow_visual_processing && !d.may_change_filter);
  d = m.onVioResult(healthy(), 10., true);
  assert(d.event_ignored && !d.has_retained_update);
}

static void distinct_service_input_permission_and_updates()
{
  TestManager m(enabled());
  auto c = healthy();
  c.last_legal_image_wall = noTime();
  auto d = m.poll(c);
  assert(d.service_alive && !d.image_fresh && !d.allow_visual_processing);
  assert(!d.has_vio_attempt && !d.has_retained_update);
  c.last_legal_image_wall = c.wall_now;
  d = m.poll(c);
  assert(d.state == State::PROBING && d.allow_visual_processing);
  assert(d.may_change_filter && d.probing_changes_filter && !d.has_retained_update);
  d = m.onVioResult(c, c.sensor_now, false);
  assert(d.has_vio_attempt && !d.has_retained_update && !d.last_attempt_retained);
  assert(d.state == State::PROBING && d.probe_calls == 1);
}

static void recovery_requires_count_and_span_of_consecutive_actual_updates()
{
  TestManager m(enabled());
  m.poll(healthy(0.));
  Decision d;
  for (int i = 0; i < 7; ++i)
  {
    const double stamp = i / 30.;
    d = m.onVioResult(healthy(stamp), stamp, true);
    if (i < 6) assert(d.state == State::PROBING);
  }
  assert(d.state == State::FUSING && d.consecutive_successes == 7);
  assert(d.has_retained_update && !d.probing_changes_filter);

  TestManager interrupted(enabled());
  interrupted.poll(healthy(0.));
  interrupted.onVioResult(healthy(0.), 0., true);
  interrupted.onVioResult(healthy(.125), .125, true);
  d = interrupted.onVioResult(healthy(.25), .25, false);
  assert(d.consecutive_successes == 0 && d.state == State::PROBING);
  interrupted.onVioResult(healthy(.375), .375, true);
  d = interrupted.onVioResult(healthy(.5), .5, true);
  assert(d.state == State::PROBING && d.consecutive_successes == 2);
  d = interrupted.onVioResult(healthy(.625), .625, true);
  assert(d.state == State::FUSING);
}

static void isolated_failure_and_nonscheduled_time_are_not_observation_loss()
{
  TestManager m(enabled());
  fuse(m);
  auto d = m.onVioResult(healthy(10.5), 10.5, false);
  assert(d.state == State::FUSING && d.consecutive_failures == 1 && !d.withdraw);
  // Legal images and source remain fresh, but no VIO call was delivered.
  // This is scheduling evidence, not six failed VIO calls.
  d = m.poll(healthy(30.));
  assert(d.state == State::FUSING && d.consecutive_failures == 1);
  assert(!d.last_attempt_retained && d.last_retained_update_sensor == 10.25);
  d = m.onVioResult(healthy(30.), 30., true);
  assert(d.state == State::FUSING && d.consecutive_failures == 0);
}

static void sustained_failure_needs_both_count_and_sensor_span()
{
  TestManager m(enabled());
  fuse(m, 0.);
  Decision d;
  for (int i = 0; i < 16; ++i)
  {
    const double stamp = 1. + i / 30.;
    d = m.onVioResult(healthy(stamp), stamp, false);
    if (i < 15) assert(d.state == State::FUSING && !d.clear_visual_cache);
  }
  assert(d.state == State::DEGRADED && d.clear_visual_cache && d.withdraw);
  assert(d.consecutive_failures == 16 && isReason(d, "sustained_actual_vio_failures"));
  const double until = d.cooldown_until;
  d = m.poll(healthy(1.6));
  assert(d.state == State::DEGRADED && !d.clear_visual_cache && !d.withdraw);
  assert(d.cooldown_until == until && d.consecutive_failures == 16);
  d = m.poll(healthy(until));
  assert(d.state == State::PROBING && d.probe_calls == 0 && d.consecutive_failures == 0);
}

static void probe_budget_then_cooldown_then_recovery()
{
  TestManager m(enabled());
  m.poll(healthy(0.));
  Decision d;
  for (int i = 0; i < 10; ++i)
  {
    const double t = i / 30.;
    d = m.onVioResult(healthy(t), t, false);
    if (i < 9) assert(d.state == State::PROBING);
  }
  assert(d.state == State::DEGRADED && d.probe_calls == 10);
  assert(isReason(d, "probe_call_budget_without_sustained_updates"));
  assert(d.clear_visual_cache && d.withdraw);
  const double until = d.cooldown_until;
  for (double t : {.4, .8, 1.2})
  {
    d = m.poll(healthy(t));
    assert(d.state == State::DEGRADED && d.probe_attempt == 1 && d.cooldown_until == until);
    assert(!d.clear_visual_cache);
  }
  d = m.poll(healthy(until));
  assert(d.state == State::PROBING && d.probe_attempt == 2 && d.probe_calls == 0);
  for (double delta : {0., .125, .25})
    d = m.onVioResult(healthy(until+delta), until+delta, true);
  assert(d.state == State::FUSING);
}

static void last_permitted_probe_call_can_recover()
{
  TestManager m(enabled());
  m.poll(healthy(0.));
  Decision d;
  for (int i = 0; i < 7; ++i) d = m.onVioResult(healthy(i*.05), i*.05, false);
  for (double t : {.5, .625, .75}) d = m.onVioResult(healthy(t), t, true);
  assert(d.probe_calls == 10 && d.state == State::FUSING);
  assert(!d.withdraw && !d.clear_visual_cache);
}

static void no_scheduling_probe_timeout_is_not_bad_observations()
{
  TestManager m(enabled());
  m.poll(healthy(10.));
  auto d = m.poll(healthy(12.));
  assert(d.state == State::DEGRADED && d.probe_calls == 0 && d.consecutive_failures == 0);
  assert(!d.has_vio_attempt && isReason(d, "probe_time_budget_no_vio_calls"));
  assert(d.withdraw && d.clear_visual_cache);
  d = m.poll(healthy(12.5));
  assert(d.probe_attempt == 1 && !d.withdraw && !d.clear_visual_cache);
  d = m.poll(healthy(13.));
  assert(d.state == State::PROBING && d.probe_attempt == 2);
}

static void service_and_image_loss_withdraw_once_and_recover_via_probe()
{
  TestManager m(enabled());
  fuse(m);
  auto c = healthy(10.5);
  c.service_fresh = false;
  auto d = m.poll(c);
  assert(d.state == State::OFF && d.image_fresh && !d.service_alive);
  assert(d.withdraw && d.clear_visual_cache && isReason(d, "service_unavailable"));
  d = m.poll(c);
  assert(!d.withdraw && !d.clear_visual_cache);
  c.service_fresh = true;
  c.wall_now = 1002.; // Heartbeat alone cannot renew image freshness.
  d = m.poll(c);
  assert(d.state == State::OFF && d.service_alive && !d.image_fresh);
  assert(isReason(d, "image_unavailable_or_stale"));
  c.last_legal_image_wall = c.wall_now;
  d = m.poll(c);
  assert(d.state == State::PROBING && d.probe_calls == 0 && d.consecutive_successes == 0);
  assert(!d.has_vio_attempt || d.last_vio_sensor == 10.25);
  for (double delta : {0., .125, .25})
  {
    c.sensor_now = 10.5+delta;
    d = m.onVioResult(c, c.sensor_now, true);
  }
  assert(d.state == State::FUSING);
  c.wall_now += 1.; // At exactly 1 second image age, no longer fresh.
  d = m.poll(c);
  assert(d.state == State::OFF && d.withdraw && d.clear_visual_cache);
}

static void frozen_sensor_time_uses_wall_freshness_but_not_fake_evidence()
{
  TestManager m(enabled());
  auto c = healthy(10., 1000.);
  m.poll(c);
  for (int i=1; i<=20; ++i)
  {
    c.wall_now = 1000.+i*.025;
    const auto d = m.poll(c);
    assert(d.state == State::PROBING && d.probe_calls == 0 && !d.has_retained_update);
  }
  c.wall_now = 1001.;
  const auto d = m.poll(c);
  assert(d.state == State::OFF && isReason(d, "image_unavailable_or_stale"));
  assert(d.withdraw && d.clear_visual_cache);
}

static void cooldown_survives_input_flapping()
{
  TestManager m(enabled());
  m.poll(healthy(10.));
  auto d = m.poll(healthy(12.));
  const double until=d.cooldown_until;
  auto c=healthy(12.1);c.service_fresh=false;
  d=m.poll(c);assert(d.state==State::OFF);
  c=healthy(12.2);d=m.poll(c);
  assert(d.state==State::DEGRADED && d.cooldown_until==until && d.probe_attempt==1);
  d=m.poll(healthy(13.));assert(d.state==State::PROBING && d.probe_attempt==2);
}

static void epoch_changes_and_clock_resets_require_new_evidence()
{
  TestManager m(enabled());
  fuse(m);
  auto c=healthy(10.5);c.model_epoch=2;
  auto d=m.onVioResult(c,10.5,true);
  assert(d.state==State::OFF && d.event_ignored && d.withdraw && d.clear_visual_cache);
  assert(!d.has_retained_update && !d.has_vio_attempt && d.consecutive_successes==0);
  d=m.poll(c);assert(d.state==State::PROBING);
  for(double delta:{0.,.125,.25})
  {
    c.sensor_now=10.5+delta;d=m.onVioResult(c,c.sensor_now,true);
  }
  assert(d.state==State::FUSING);
  c.sensor_now=1.;d=m.poll(c);
  assert(d.state==State::OFF && isReason(d,"sensor_time_reset") && d.clear_visual_cache);
  assert(!d.has_retained_update && d.probe_attempt==0);
  d=m.poll(c);assert(d.state==State::PROBING);
  c.service_fresh=false;m.poll(c);
  c.model_epoch=3;d=m.poll(c);
  assert(d.state==State::OFF && !d.changed && d.clear_visual_cache);
}

static void stale_duplicate_or_unscheduled_result_never_counts_as_recovery()
{
  TestManager m(enabled());
  auto c=healthy(10.);
  auto d=m.onVioResult(c,10.,true);
  assert(d.state==State::PROBING && d.event_ignored && !d.has_retained_update);
  d=m.onVioResult(c,10.,true);assert(d.consecutive_successes==1);
  d=m.onVioResult(c,10.,true);assert(d.event_ignored && d.consecutive_successes==1);
  c.sensor_now=10.1;
  d=m.onVioResult(c,9.9,true);assert(d.event_ignored && d.consecutive_successes==1);
  d=m.onVioResult(c,std::numeric_limits<double>::quiet_NaN(),true);
  assert(d.event_ignored && d.consecutive_successes==1);
}

static void model_availability_changes_require_new_epoch_and_image_evidence()
{
  TestManager m(enabled());
  fuse(m);
  auto c=healthy(10.5);
  c.model_usable=false; // Service is alive, but the supplied fusion model is unusable.
  auto d=m.poll(c);
  assert(d.state==State::OFF && d.service_alive && isReason(d,"model_unusable"));
  assert(d.withdraw && d.clear_visual_cache);
  c.model_usable=true;
  c.model_epoch=2;
  c.last_legal_image_wall=noTime(); // Adapter invalidates pre-epoch image freshness.
  d=m.poll(c);
  assert(d.state==State::OFF && !d.has_retained_update && d.clear_visual_cache);
  d=m.poll(c);
  assert(d.state==State::OFF && isReason(d,"image_unavailable_or_stale"));
  c.sensor_now=10.75;c.last_legal_image_wall=c.wall_now;
  d=m.poll(c);
  assert(d.state==State::PROBING && d.consecutive_successes==0);
  d=m.onVioResult(c,10.75,true);
  assert(d.state==State::PROBING && d.consecutive_successes==1);
}

int main()
{
  const Config defaults;
  assert(defaults.enabled && defaults.feedback_mode == visual_feedback::Mode::MeasurementCommitV1);
  assert(defaults.recovery_schedule == RecoverySchedule::Burst);
  explicitly_disabled();
  distinct_service_input_permission_and_updates();
  recovery_requires_count_and_span_of_consecutive_actual_updates();
  isolated_failure_and_nonscheduled_time_are_not_observation_loss();
  sustained_failure_needs_both_count_and_sensor_span();
  probe_budget_then_cooldown_then_recovery();
  last_permitted_probe_call_can_recover();
  no_scheduling_probe_timeout_is_not_bad_observations();
  service_and_image_loss_withdraw_once_and_recover_via_probe();
  frozen_sensor_time_uses_wall_freshness_but_not_fake_evidence();
  cooldown_survives_input_flapping();
  epoch_changes_and_clock_resets_require_new_evidence();
  stale_duplicate_or_unscheduled_result_never_counts_as_recovery();
  model_availability_changes_require_new_epoch_and_image_evidence();
  std::cout << "visual_lifecycle: 14 scenario groups passed; synthetic policy only, no real visual recovery claim\n";
}
