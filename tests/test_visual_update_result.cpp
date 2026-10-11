// Small read-only evidence/policy regression. No ROS, sensor simulator, or
// manufactured production valid state. Production solver integration is separate.
#include <Eigen/Core>
#include "visual_update_result.h"
#include "visual_lifecycle.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

using visual_feedback::Tracker;
using visual_feedback::Result;
using visual_feedback::Mode;
using visual_lifecycle::Manager;
using visual_lifecycle::State;

static Eigen::MatrixXd poseH()
{
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, 7);
  h(0, 0) = 2.; h(1, 5) = -3.;
  return h;
}

static void accept(Tracker &tracker, const Eigen::MatrixXd &h, double residual = 0.)
{
  tracker.observeLinearization(h, Eigen::VectorXd::Constant(h.rows(), residual), h.rows());
  tracker.accepted(true, true);
}

static void complete(Tracker &tracker)
{
  tracker.covarianceCommit(true);
  tracker.finish(true);
}

static Result oneResult(const Eigen::MatrixXd &h, double residual = 0.)
{
  Tracker tracker;
  tracker.reset();
  accept(tracker, h, residual);
  complete(tracker);
  return tracker.result();
}

static void exact_measurement_evidence()
{
  const auto normal = oneResult(poseH(), 5.);
  assert(normal.eligible(true));
  // Zero innovation produces zero mean correction but remains usable evidence.
  const auto zero_innovation = oneResult(poseH());
  assert(zero_innovation.eligible(true));
  assert(!zero_innovation.eligible(false)); // legacy additional guard preserved

  const auto zero = oneResult(Eigen::MatrixXd::Zero(2, 7), 5.);
  assert(zero.path.surviving_accepts == 1 && !zero.path.pose_information);
  assert(zero.covariance_committed && !zero.eligible(true));
  assert(std::strcmp(zero.reason(true), "no_surviving_pose_information") == 0);

  auto exposure = Eigen::MatrixXd::Zero(2, 7).eval();
  exposure(0, 6) = 1000.;
  const auto e = oneResult(exposure);
  assert(e.path.exposure_information && !e.path.pose_information && !e.eligible(true));

  // Inverse composition has six actual columns and no exposure measurement.
  assert(oneResult(poseH().leftCols(6)).eligible(true));
  auto tiny = Eigen::MatrixXd::Zero(1, 6).eval();
  tiny(0, 0) = std::numeric_limits<double>::denorm_min();
  assert(oneResult(tiny).eligible(true)); // no empirical epsilon or square underflow
}

static void retained_path_not_ever_seen_information()
{
  Tracker t;
  // Iterated updates are relative to the propagated prior, not independent
  // increments: a later accepted zero-H candidate can return to that prior.
  // Earlier nonzero H must therefore not manufacture final measurement evidence.
  t.reset();
  accept(t, poseH());
  accept(t, Eigen::MatrixXd::Zero(2, 7));
  complete(t);
  assert(t.result().path.surviving_accepts == 2);
  assert(!t.result().path.pose_information && !t.result().eligible(true));

  t.reset();
  accept(t, poseH());
  auto exposure_only = Eigen::MatrixXd::Zero(2, 7).eval();
  exposure_only(0, 6) = 2.;
  accept(t, exposure_only);
  complete(t);
  assert(t.result().path.exposure_information && !t.result().path.pose_information);
  assert(!t.result().eligible(true));

  t.reset();
  accept(t, Eigen::MatrixXd::Zero(2, 7));
  const auto zero_path = t.snapshot();
  accept(t, poseH());
  assert(t.result().path.pose_information);
  t.restore(zero_path);
  complete(t);
  assert(t.result().accepted_attempts == 2 && t.result().rollback_count == 1);
  assert(t.result().path.surviving_accepts == 1 && !t.result().path.pose_information);
  assert(t.result().covariance_basis_pose_information);
  assert(!t.result().covariance_basis_survives);
  assert(!t.result().eligible(true)); // original G can still be nonzero; no success

  t.reset();
  const auto empty = t.snapshot();
  accept(t, poseH());
  t.restore(empty);
  complete(t);
  assert(t.result().path.surviving_accepts == 0);
  assert(!t.result().eligible(true));

  t.reset();
  accept(t, poseH());
  const auto good_path = t.snapshot();
  accept(t, Eigen::MatrixXd::Zero(2, 7));
  t.restore(good_path);
  complete(t);
  assert(t.result().path.surviving_accepts == 1 && t.result().path.pose_information);
  assert(!t.result().covariance_basis_survives && t.result().eligible(true));
  // The latest G's origin is diagnostic, not a covert new filter acceptance rule.

  t.reset();
  t.observeLinearization(poseH(), Eigen::VectorXd::Zero(2), 2);
  t.restore(t.snapshot()); // rejected before acceptance; evidence never retained
  complete(t);
  assert(t.result().attempted_linearizations == 1 && !t.result().eligible(true));
}

static void nonfinite_and_call_boundaries()
{
  Tracker t;
  assert(!t.result().executed && !t.result().eligible(true));
  t.reset();
  accept(t, poseH());
  assert(!t.result().eligible(true)); // completion and actual P commit mandatory
  t.finish(true);
  assert(!t.result().eligible(true));

  t.reset();
  auto invalid = poseH();
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const auto empty = t.snapshot();
  t.observeLinearization(invalid, Eigen::VectorXd::Zero(2), 2);
  t.accepted(false, false);
  t.restore(empty);
  accept(t, poseH());
  complete(t);
  assert(t.result().path.pose_information && t.result().nonfinite_attempt);
  assert(!t.result().eligible(true)); // rollback cannot erase numerical failure

  t.reset();
  t.observeLinearization(poseH(), Eigen::VectorXd::Constant(2, 1e100), 2, false);
  t.accepted(true, true); // actual error accumulator nonfinite although H/z finite
  complete(t);
  assert(t.result().nonfinite_attempt && !t.result().eligible(true));

  t.reset();
  accept(t, poseH());
  t.covarianceCommit(false);
  t.finish(true);
  assert(!t.result().eligible(true));
  t.reset();
  accept(t, poseH());
  t.covarianceCommit(true);
  t.finish(false);
  assert(!t.result().eligible(true));

  t.reset();
  t.observeLinearization(Eigen::MatrixXd::Zero(2, 5), Eigen::VectorXd::Zero(2), 2);
  t.accepted(true, true);
  complete(t);
  assert(t.result().invalid_measurement_shape && !t.result().eligible(true));

  t.reset(); // repeated normal call must not inherit failure, G basis, or completion
  assert(!t.result().nonfinite_attempt && !t.result().invalid_measurement_shape);
  assert(!t.result().completed && t.result().path.surviving_accepts == 0);
  assert(!t.result().covariance_committed && t.result().covariance_basis_step == 0);
  accept(t, poseH()); complete(t);
  assert(t.result().eligible(true));

  t.reset();
  t.finish(true); // no-support processFrame: real call, no pretend linearization
  assert(t.result().executed && t.result().completed);
  assert(t.result().attempted_linearizations == 0 && !t.result().eligible(true));
}

static visual_lifecycle::Context context(double stamp, std::uint64_t epoch = 1)
{
  visual_lifecycle::Context c;
  c.wall_now = 1000. + stamp;
  c.sensor_now = stamp;
  c.last_legal_image_wall = c.wall_now;
  c.service_fresh = c.model_usable = c.lio_ready = true;
  c.model_epoch = epoch;
  return c;
}

static visual_lifecycle::Event event(std::uint64_t id, double stamp,
                                     const Result &r, std::uint64_t epoch = 1)
{
  visual_lifecycle::Event e;
  e.call_id = id; e.sensor_stamp = stamp; e.model_epoch = epoch;
  e.executed = true; e.legacy_retained = true; e.result = r;
  return e;
}

static Manager manager(Mode mode)
{
  visual_lifecycle::Config c;
  c.enabled = true; c.feedback_mode = mode;
  return Manager(c);
}

static void policy_counts_and_identity()
{
  const Result good = oneResult(poseH());
  const Result zero = oneResult(Eigen::MatrixXd::Zero(2, 7));
  auto m = manager(Mode::MeasurementCommitV1);
  m.poll(context(10.));
  auto d = m.onVioResult(context(10.), event(1, 10., good));
  assert(d.consecutive_successes == 1);
  const double success_stamp = d.last_retained_update_sensor;
  d = m.onVioResult(context(10.125), event(2, 10.125, zero));
  assert(d.consecutive_failures == 1 && d.consecutive_successes == 0);
  assert(d.last_retained_update_sensor == success_stamp && d.probe_calls == 2);
  d = m.onVioResult(context(10.25), event(3, 10.25, zero));
  assert(d.consecutive_failures == 2 && d.probe_calls == 3);
  assert(d.last_retained_update_sensor == success_stamp);
  d = m.onVioResult(context(10.25), event(3, 10.25, good));
  assert(d.event_ignored && d.consecutive_failures == 2 && d.probe_calls == 3);
  d = m.onVioResult(context(10.3), event(4, 10.125, good));
  assert(d.event_ignored && d.consecutive_failures == 2 && d.probe_calls == 3);
  d = m.onVioResult(context(10.3), event(5, 10.3, good, 0));
  assert(d.event_ignored && d.consecutive_failures == 2 && d.probe_calls == 3);
  auto unexecuted = event(6, 10.3, good);
  unexecuted.executed = false;
  d = m.onVioResult(context(10.3), unexecuted);
  assert(d.event_ignored && d.consecutive_failures == 2 && d.probe_calls == 3);

  for (int i = 0; i < 3; ++i)
    d = m.onVioResult(context(10.5 + i*.125), event(7+i, 10.5+i*.125, good));
  assert(d.state == State::FUSING && d.consecutive_successes == 3);
  assert(d.last_retained_update_sensor == 10.75);
  // Exactly one failed attempt per completed call, keeping original six/.5 rule.
  for (int i = 0; i < 6; ++i)
    d = m.onVioResult(context(11.+i*.125), event(10+i, 11.+i*.125, zero));
  assert(d.state == State::DEGRADED && d.consecutive_failures == 6);
  assert(d.last_retained_update_sensor == 10.75);

  auto legacy = manager(Mode::LegacyFinite);
  legacy.poll(context(10.));
  for (int i = 0; i < 3; ++i)
    d = legacy.onVioResult(context(10.+i*.125), event(1+i, 10.+i*.125, zero));
  assert(d.state == State::FUSING); // frozen compatibility retains original meaning

  auto no_fallback = manager(Mode::MeasurementCommitV1);
  no_fallback.poll(context(10.));
  d = no_fallback.onVioResult(context(10.), 10., true);
  assert(d.event_ignored && d.probe_calls == 0 && d.consecutive_successes == 0);
}

static void environment_and_epoch_guards_remain_active()
{
  auto m = manager(Mode::MeasurementCommitV1);
  const auto good = oneResult(poseH());
  m.poll(context(10.));
  m.onVioResult(context(10.), event(1, 10., good));
  auto d = m.onVioResult(context(10.125, 2), event(2, 10.125, good, 1));
  assert(d.event_ignored && d.state == State::OFF);
  assert(d.consecutive_successes == 0 && d.consecutive_failures == 0 && d.probe_calls == 0);
  m.poll(context(10.25, 2));
  d = m.onVioResult(context(10.25, 2), event(3, 10.25, good, 1));
  assert(d.event_ignored && d.consecutive_successes == 0 && d.probe_calls == 0);
  d = m.onVioResult(context(10.375, 2), event(4, 10.375, good, 2));
  assert(d.consecutive_successes == 1 && d.probe_calls == 1);
  auto stale = context(10.5, 2);
  stale.last_legal_image_wall -= 2.;
  d = m.onVioResult(stale, event(5, 10.5, good, 2));
  assert(d.event_ignored && d.state == State::OFF && d.probe_calls == 1);
  assert(d.consecutive_successes == 1); // rejected event adds neither success nor failure
}

int main()
{
  assert(visual_feedback::parseMode("legacy_finite") == Mode::LegacyFinite);
  assert(visual_feedback::parseMode("measurement_commit_v1") == Mode::MeasurementCommitV1);
  bool invalid_mode = false;
  try { visual_feedback::parseMode("guess"); } catch (const std::invalid_argument &) { invalid_mode = true; }
  assert(invalid_mode);
  exact_measurement_evidence();
  retained_path_not_ever_seen_information();
  nonfinite_and_call_boundaries();
  policy_counts_and_identity();
  environment_and_epoch_guards_remain_active();
  std::cout << "visual_update_result: evidence, rollback, call identity and policy regressions passed\n";
}
