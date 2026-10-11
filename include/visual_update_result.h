#pragma once

// Read-only evidence about one actual VIO call. This does not extend the filter
// state, alter a solver decision, or certify correspondence quality/accuracy.
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace visual_feedback
{
enum class Mode { LegacyFinite, MeasurementCommitV1 };

inline const char *modeName(Mode mode)
{
  return mode == Mode::MeasurementCommitV1 ? "measurement_commit_v1" : "legacy_finite";
}

inline Mode parseMode(const std::string &name)
{
  if (name == "legacy_finite") return Mode::LegacyFinite;
  if (name == "measurement_commit_v1") return Mode::MeasurementCommitV1;
  throw std::invalid_argument("Unknown visual feedback mode: " + name);
}

// This value is copied beside each production old_state snapshot. Attempt
// diagnostics and the actual covariance basis deliberately do NOT roll back.
struct Path
{
  // Diagnostic count along the retained state history, not additive independent
  // measurement information. The flags describe the current retained candidate.
  std::uint64_t surviving_accepts = 0;
  std::uint64_t last_step = 0;
  bool pose_information = false;
  bool exposure_information = false;
};

struct Result
{
  bool executed = false;
  bool completed = false;
  std::uint64_t attempted_linearizations = 0;
  std::uint64_t accepted_attempts = 0;
  std::uint64_t rollback_count = 0;
  std::uint64_t last_measurement_rows = 0;
  bool nonfinite_attempt = false;
  bool invalid_measurement_shape = false;
  bool last_measurement_finite = false;
  bool last_pose_information = false;
  bool last_exposure_information = false;
  Path path;
  // The solver's latest G is NOT restored by its mean-state rollback. Preserve
  // that fact; covariance-only information cannot manufacture a retained path.
  std::uint64_t covariance_basis_step = 0;
  bool covariance_basis_pose_information = false;
  bool covariance_basis_survives = false;
  bool covariance_committed = false;
  bool covariance_finite = false;
  bool final_finite = false;

  const char *reason(bool legacy_retained) const
  {
    if (!executed) return "not_executed";
    if (!completed) return "incomplete_call";
    if (invalid_measurement_shape) return "invalid_measurement_shape";
    if (nonfinite_attempt || !final_finite) return "nonfinite_visual_result";
    if (attempted_linearizations == 0) return "no_measurements";
    if (!legacy_retained) return "legacy_not_retained";
    if (path.surviving_accepts == 0) return "no_surviving_accept";
    if (!path.pose_information) return "no_surviving_pose_information";
    if (!covariance_committed) return "covariance_not_committed";
    if (!covariance_finite) return "nonfinite_covariance_commit";
    return "eligible_measurement_commit";
  }

  bool eligible(bool legacy_retained) const
  {
    return executed && completed && !invalid_measurement_shape &&
      !nonfinite_attempt && final_finite && legacy_retained &&
      path.surviving_accepts > 0 && path.pose_information &&
      covariance_committed && covariance_finite;
  }
};

class Tracker
{
public:
  // Call immediately on entry, including paths that can return early.
  void reset()
  {
    result_ = Result();
    result_.executed = true;
    current_usable_ = false;
  }

  Path snapshot() const { return result_.path; }

  void restore(const Path &path)
  {
    result_.path = path;
    ++result_.rollback_count;
    updateBasisSurvival();
  }

  // Inspect exactly the H/z which the production solver is about to use.
  // State mapping: columns 0..2 rotation perturbation (rad), 3..5 position (m),
  // optional 6 inverse exposure. The remaining state columns are absent from
  // this direct visual measurement. A finite exactly nonzero pose coefficient
  // is the minimal evidence, NOT a rank/quality threshold or comparison across
  // unlike units. No squaring (under/overflow), epsilon, eigenvalue or prior.
  template <typename HType, typename ZType>
  void observeLinearization(const HType &h, const ZType &z, int n_meas,
                            bool evaluation_finite = true)
  {
    ++result_.attempted_linearizations;
    result_.last_measurement_rows = n_meas > 0 ? static_cast<std::uint64_t>(n_meas) : 0;
    result_.last_pose_information = false;
    result_.last_exposure_information = false;
    const bool shape = h.cols() >= 6 && h.rows() == z.size() &&
                       n_meas >= 0 && n_meas <= h.rows();
    // Include the actual scalar error used in the acceptance comparison: the
    // production float accumulation may overflow even when H/z remain finite.
    const bool finite = h.allFinite() && z.allFinite() && evaluation_finite;
    result_.last_measurement_finite = shape && finite;
    result_.invalid_measurement_shape |= !shape;
    result_.nonfinite_attempt |= !finite;
    current_usable_ = shape && finite && n_meas > 0;
    if (!current_usable_) return;
    result_.last_pose_information = (h.leftCols(6).array() != 0.).any();
    if (h.cols() > 6)
      result_.last_exposure_information = (h.col(6).array() != 0.).any();
  }

  // Call after the actual accepted-branch state application, even if nonfinite.
  // The caller captures snapshot() exactly when it copies production old_state.
  void accepted(bool solution_finite, bool state_finite)
  {
    ++result_.accepted_attempts;
    result_.nonfinite_attempt |= !solution_finite || !state_finite;
    result_.covariance_basis_step = result_.accepted_attempts;
    result_.covariance_basis_pose_information = current_usable_ && result_.last_pose_information;
    if (current_usable_ && solution_finite && state_finite)
    {
      ++result_.path.surviving_accepts;
      result_.path.last_step = result_.accepted_attempts;
      // Production iterated updates include propagated_state - current_state;
      // successive candidates are not independent accumulated measurements.
      // In particular, accepting H=0 can return the mean to its propagated prior.
      // Replace the candidate's evidence; old_state rollback restores its exact
      // previous snapshot rather than retaining an "ever informative" flag.
      result_.path.pose_information = result_.last_pose_information;
      result_.path.exposure_information = result_.last_exposure_information;
    }
    updateBasisSurvival();
  }

  void covarianceCommit(bool finite)
  {
    result_.covariance_committed = true;
    result_.covariance_finite = finite;
    result_.nonfinite_attempt |= !finite;
    updateBasisSurvival();
  }

  void finish(bool final_finite)
  {
    result_.completed = true;
    result_.final_finite = final_finite;
    result_.nonfinite_attempt |= !final_finite;
    updateBasisSurvival();
  }

  const Result &result() const { return result_; }

private:
  void updateBasisSurvival()
  {
    result_.covariance_basis_survives = result_.covariance_basis_step > 0 &&
      result_.covariance_basis_step == result_.path.last_step;
  }

  Result result_;
  bool current_usable_ = false;
};
} // namespace visual_feedback
