#pragma once

// Optional frontend management policy, independent of ROS and the filter.
// All methods run on one caller-owned thread. This class only requests actions;
// it neither applies model parameters nor clears caches nor changes filter state.
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "visual_update_result.h"

namespace visual_lifecycle
{
enum class State { OFF, PROBING, FUSING, DEGRADED };
enum class RecoverySchedule { Burst };

inline const char *recoveryScheduleName(RecoverySchedule schedule)
{
  (void)schedule;
  return "burst";
}

inline RecoverySchedule parseRecoverySchedule(const std::string &name)
{
  if (name == "burst") return RecoverySchedule::Burst;
  throw std::invalid_argument("Unknown visual recovery schedule: " + name);
}

inline const char *stateName(State state)
{
  switch (state)
  {
    case State::OFF: return "OFF";
    case State::PROBING: return "PROBING";
    case State::FUSING: return "FUSING";
    case State::DEGRADED: return "DEGRADED";
  }
  return "UNKNOWN";
}

inline double noTime() { return -std::numeric_limits<double>::infinity(); }

struct Config
{
  bool enabled = true;
  visual_feedback::Mode feedback_mode = visual_feedback::Mode::MeasurementCommitV1;
  RecoverySchedule recovery_schedule = RecoverySchedule::Burst;
  double image_fresh_wall_s = 1.0;
  unsigned failure_count = 6;
  double failure_span_sensor_s = 0.5;
  unsigned recovery_count = 3;
  double recovery_span_sensor_s = 0.2;
  unsigned probe_call_budget = 10;
  double probe_span_sensor_s = 2.0;
  double cooldown_sensor_s = 1.0;

  void validate() const
  {
    if (failure_count == 0 || recovery_count == 0 || probe_call_budget == 0 ||
        recovery_count > probe_call_budget)
      throw std::invalid_argument("Invalid visual lifecycle call budget");
    const double times[] = {image_fresh_wall_s, failure_span_sensor_s,
      recovery_span_sensor_s, probe_span_sensor_s, cooldown_sensor_s};
    for (double time : times)
      if (!std::isfinite(time) || time <= 0.)
        throw std::invalid_argument("Visual lifecycle time limits must be finite and positive");
  }
};

struct Context
{
  double wall_now = 0.;
  double sensor_now = 0.;
  // Update this only after contract checks AND successful image decoding,
  // independently of current processing permission. A message header is not
  // sufficient. Duplicate/stale sensor images must not refresh this timestamp.
  double last_legal_image_wall = noTime();
  bool service_fresh = false;
  bool model_usable = false;
  bool lio_ready = false;
  // Caller assigns an epoch for provider session/model revision changes.
  // The provider's revision and the frontend's epoch are distinct identities.
  std::uint64_t model_epoch = 0;
};

struct Decision
{
  State previous = State::OFF;
  State state = State::OFF;
  bool changed = false;
  const char *reason = "disabled";
  bool service_alive = false;
  bool image_fresh = false;
  bool model_usable = false;
  bool lio_ready = false;
  bool allow_visual_processing = false;
  // PROBING uses the real VIO/filter. It is NOT an isolated dry run.
  bool may_change_filter = false;
  bool probing_changes_filter = false;
  bool clear_visual_cache = false;
  bool withdraw = false;
  bool event_ignored = false;
  const char *event_reason = "none";
  unsigned probe_calls = 0;
  std::uint64_t probe_attempt = 0;
  std::uint64_t consecutive_failures = 0;
  std::uint64_t consecutive_successes = 0;
  double probe_started_sensor = noTime();
  double cooldown_until = noTime();
  bool has_vio_attempt = false;
  double last_vio_sensor = noTime();
  bool last_attempt_retained = false;
  bool has_retained_update = false;
  double last_retained_update_sensor = noTime();
  std::uint64_t model_epoch = 0;
};

// Identity is captured before an actual call; evidence is copied at completion.
// call_id must be monotonic for the mapper lifetime, including visual clearing.
struct Event
{
  std::uint64_t call_id = 0;
  std::uint64_t model_epoch = 0;
  double sensor_stamp = noTime();
  bool executed = false;
  bool legacy_retained = false;
  visual_feedback::Result result;
};

class Manager
{
public:
  explicit Manager(Config config = Config()) : config_(config) { config_.validate(); }

  State state() const { return state_; }
  const Config &config() const { return config_; }

  Decision poll(const Context &context) { return advance(context, false, noTime(), false); }

  // Historical compatibility only: this boolean follows the original finite
  // accepted-branch predicate; it does NOT establish visual measurement health.
  Decision onVioResult(const Context &context, double sensor_stamp, bool retained_update)
  {
    if (config_.feedback_mode != visual_feedback::Mode::LegacyFinite)
      return ignoreEvent(context, "structured_feedback_required");
    return advance(context, true, sensor_stamp, retained_update);
  }

  Decision onVioResult(const Context &context, const Event &event)
  {
    if (!event.executed || !event.result.executed)
      return ignoreEvent(context, "vio_not_executed");
    if (event.call_id == 0 || event.call_id <= last_event_call_id_)
      return ignoreEvent(context, "duplicate_or_old_call_id");
    // Once delivered, an event can never become fresh merely by being retried.
    last_event_call_id_ = event.call_id;
    if (event.model_epoch != context.model_epoch)
      return ignoreEvent(context, "result_model_epoch_mismatch");
    const bool retained = config_.feedback_mode == visual_feedback::Mode::LegacyFinite ?
      event.legacy_retained : event.result.eligible(event.legacy_retained);
    Decision d = advance(context, true, event.sensor_stamp, retained);
    if (!d.event_ignored)
      d.event_reason = config_.feedback_mode == visual_feedback::Mode::LegacyFinite ?
        (retained ? "legacy_retained_finite" : "legacy_not_retained") :
        event.result.reason(event.legacy_retained);
    return d;
  }

private:
  static bool active(State state) { return state == State::PROBING || state == State::FUSING; }

  Decision ignoreEvent(const Context &context, const char *reason)
  {
    // Environmental and elapsed-time guards still run. The rejected event itself
    // contributes neither an update/failure nor a probe call.
    Decision d = poll(context);
    d.event_ignored = true;
    d.event_reason = reason;
    return d;
  }

  void resetHistory()
  {
    probe_calls_ = 0;
    probe_attempt_ = 0;
    failures_ = successes_ = 0;
    first_failure_ = first_success_ = probe_started_ = cooldown_until_ = noTime();
    last_vio_ = last_retained_ = noTime();
    last_attempt_retained_ = false;
  }

  void startProbe(double sensor_now, const char *reason)
  {
    state_ = State::PROBING;
    reason_ = reason;
    probe_started_ = sensor_now;
    probe_calls_ = 0;
    ++probe_attempt_;
    failures_ = successes_ = 0;
    first_failure_ = first_success_ = noTime();
  }

  void degrade(double sensor_now, const char *reason)
  {
    state_ = State::DEGRADED;
    reason_ = reason;
    cooldown_until_ = sensor_now + config_.cooldown_sensor_s;
    // Preserve the completed attempt's counters for diagnostics. They reset
    // only on a real new probe, never on each idle poll.
  }

  Decision snapshot(const Context &context, State previous, bool image_fresh,
                    bool clear, bool withdraw, bool ignored, const char *event_reason) const
  {
    Decision d;
    d.previous = previous;
    d.state = state_;
    d.changed = previous != state_;
    d.reason = reason_;
    d.service_alive = context.service_fresh;
    d.image_fresh = image_fresh;
    d.model_usable = context.model_usable;
    d.lio_ready = context.lio_ready;
    d.allow_visual_processing = active(state_);
    d.may_change_filter = d.allow_visual_processing;
    d.probing_changes_filter = state_ == State::PROBING;
    d.clear_visual_cache = clear;
    d.withdraw = withdraw;
    d.event_ignored = ignored;
    d.event_reason = event_reason;
    d.probe_calls = probe_calls_;
    d.probe_attempt = probe_attempt_;
    d.consecutive_failures = failures_;
    d.consecutive_successes = successes_;
    d.probe_started_sensor = probe_started_;
    d.cooldown_until = cooldown_until_;
    d.has_vio_attempt = std::isfinite(last_vio_);
    d.last_vio_sensor = last_vio_;
    d.last_attempt_retained = last_attempt_retained_;
    d.has_retained_update = std::isfinite(last_retained_);
    d.last_retained_update_sensor = last_retained_;
    d.model_epoch = model_epoch_;
    return d;
  }

  Decision advance(const Context &context, bool event, double stamp, bool retained)
  {
    const State previous = state_;
    const bool was_active = active(previous);
    const double age = context.wall_now - context.last_legal_image_wall;
    const bool fresh = std::isfinite(context.last_legal_image_wall) && std::isfinite(age) &&
                       age >= 0. && age < config_.image_fresh_wall_s;
    bool clear = false, withdraw = false, ignored = false;
    const char *event_reason = "none";
    auto result = [&]() { return snapshot(context, previous, fresh, clear, withdraw, ignored, event_reason); };

    if (!std::isfinite(context.wall_now) || !std::isfinite(context.sensor_now))
    {
      state_ = State::OFF;
      reason_ = "invalid_time_context";
      clear = withdraw = was_active;
      ignored = event;
      event_reason = event ? "invalid_time_context" : "none";
      return result();
    }
    const bool epoch_changed = epoch_seen_ && context.model_epoch != model_epoch_;
    const bool time_reset = std::isfinite(last_sensor_now_) && context.sensor_now < last_sensor_now_;
    model_epoch_ = context.model_epoch;
    epoch_seen_ = true;
    last_sensor_now_ = context.sensor_now;
    if (epoch_changed || time_reset)
    {
      resetHistory();
      state_ = State::OFF;
      reason_ = time_reset ? "sensor_time_reset" : "model_epoch_changed";
      // A new model epoch invalidates visual cache even if the old policy was
      // already inactive. Actual cache mutation remains the caller's job.
      clear = was_active || epoch_changed;
      withdraw = was_active;
      ignored = event;
      event_reason = event ? "result_crossed_epoch_or_time_reset" : "none";
      // Do not count an in-flight result from the previous model or time epoch.
      // The next poll can start a new probe using fresh inputs.
      return result();
    }

    const char *unavailable = !config_.enabled ? "disabled" :
      !context.service_fresh ? "service_unavailable" :
      !context.model_usable ? "model_unusable" :
      !context.lio_ready ? "lio_not_ready" :
      !fresh ? "image_unavailable_or_stale" : nullptr;
    if (unavailable)
    {
      state_ = State::OFF;
      reason_ = unavailable;
      clear = withdraw = was_active;
      ignored = event;
      event_reason = event ? "result_after_prerequisite_loss" : "none";
      // Cooldown and last consumed VIO stamp survive brief input/service loss.
      return result();
    }

    if (state_ == State::OFF || state_ == State::DEGRADED)
    {
      if (context.sensor_now >= cooldown_until_)
        startProbe(context.sensor_now, previous == State::OFF ? "probing_available_inputs" : "probing_after_cooldown");
      else if (state_ == State::OFF)
      {
        state_ = State::DEGRADED;
        reason_ = "cooldown_after_input_recovery";
      }
      // Results received while processing was disallowed are never recovery
      // evidence, even when this call has just opened a new probe window.
      if (event)
      {
        ignored = true;
        event_reason = "result_was_not_permitted_before_call";
        return result();
      }
    }

    if (event)
    {
      if (!std::isfinite(stamp) || stamp <= last_vio_ ||
          (state_ == State::PROBING && stamp < probe_started_))
      {
        ignored = true;
        event_reason = "duplicate_nonfinite_or_preprobe_vio_result";
      }
      else if (active(state_))
      {
        last_vio_ = stamp;
        last_attempt_retained_ = retained;
        if (state_ == State::PROBING) ++probe_calls_;
        if (retained)
        {
          last_retained_ = stamp;
          if (successes_ == 0) first_success_ = stamp;
          ++successes_;
          failures_ = 0;
          first_failure_ = noTime();
          if (state_ == State::PROBING && successes_ >= config_.recovery_count &&
              stamp - first_success_ >= config_.recovery_span_sensor_s)
          {
            state_ = State::FUSING;
            reason_ = "sustained_retained_visual_updates";
          }
        }
        else
        {
          if (failures_ == 0) first_failure_ = stamp;
          ++failures_;
          successes_ = 0;
          first_success_ = noTime();
          if (state_ == State::FUSING && failures_ >= config_.failure_count &&
              stamp - first_failure_ >= config_.failure_span_sensor_s)
            degrade(context.sensor_now, "sustained_actual_vio_failures");
        }
      }
    }

    // A real retained result is evaluated first, including the last permitted
    // call. An unscheduled probe can still end, without claiming bad geometry.
    if (state_ == State::PROBING)
    {
      if (probe_calls_ >= config_.probe_call_budget)
        degrade(context.sensor_now, "probe_call_budget_without_sustained_updates");
      else if (context.sensor_now - probe_started_ >= config_.probe_span_sensor_s)
        degrade(context.sensor_now, probe_calls_ == 0 ? "probe_time_budget_no_vio_calls" :
                "probe_time_budget_insufficient_update_evidence");
    }
    if (was_active && !active(state_)) clear = withdraw = true;
    return result();
  }

  Config config_;
  State state_ = State::OFF;
  const char *reason_ = "disabled";
  bool epoch_seen_ = false;
  std::uint64_t model_epoch_ = 0;
  double last_sensor_now_ = noTime();
  unsigned probe_calls_ = 0;
  std::uint64_t probe_attempt_ = 0;
  std::uint64_t failures_ = 0, successes_ = 0;
  double first_failure_ = noTime(), first_success_ = noTime();
  double probe_started_ = noTime(), cooldown_until_ = noTime();
  double last_vio_ = noTime(), last_retained_ = noTime();
  bool last_attempt_retained_ = false;
  std::uint64_t last_event_call_id_ = 0;
};
} // namespace visual_lifecycle
