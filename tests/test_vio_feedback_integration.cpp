// Functional constructions through the real VIO solver, without a ROS master,
// replay, map retrieval, or a claim about physical truth/recovery performance.
// Links the local production visual solver; no external core library is used.
#include "vio.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
void require(bool value, const std::string &message)
{
  // Deliberately independent of assert/NDEBUG in release builds.
  if (!value) throw std::runtime_error(message);
}

bool finiteState(const StatesGroup &s)
{
  return s.rot_end.allFinite() && s.pos_end.allFinite() &&
    std::isfinite(s.inv_expo_time) && s.vel_end.allFinite() &&
    s.bias_g.allFinite() && s.bias_a.allFinite() && s.gravity.allFinite() &&
    s.cov.allFinite();
}

double meanDelta(const StatesGroup &a, const StatesGroup &b)
{
  return std::max({(a.rot_end - b.rot_end).cwiseAbs().maxCoeff(),
    (a.pos_end - b.pos_end).cwiseAbs().maxCoeff(),
    std::abs(a.inv_expo_time - b.inv_expo_time),
    (a.vel_end - b.vel_end).cwiseAbs().maxCoeff(),
    (a.bias_g - b.bias_g).cwiseAbs().maxCoeff(),
    (a.bias_a - b.bias_a).cwiseAbs().maxCoeff(),
    (a.gravity - b.gravity).cwiseAbs().maxCoeff()});
}

template <class Derived>
void matrixJson(const Eigen::MatrixBase<Derived> &m)
{
  std::cout << '[';
  for (int row = 0; row < m.rows(); ++row)
    for (int col = 0; col < m.cols(); ++col)
    {
      if (row || col) std::cout << ',';
      std::cout << m(row, col);
    }
  std::cout << ']';
}

cv::Mat image(bool textured, unsigned char value = 0)
{
  cv::Mat result(64, 64, CV_8UC1, cv::Scalar(value));
  if (textured)
    for (int y = 0; y < result.rows; ++y)
      for (int x = 0; x < result.cols; ++x)
        result.at<unsigned char>(y, x) =
          static_cast<unsigned char>(16 + x + 2 * y + (x * y) % 17);
  return result;
}

struct Fixture
{
  StatesGroup state;
  StatesGroup propagated;
  VIOManager vio;
  std::unique_ptr<VisualPoint> point;

  explicit Fixture(bool inverse)
  {
    state.cov.setIdentity();
    state.cov *= 0.01;
    propagated = state;
    vio.state = &state;
    vio.state_propagat = &propagated;
    vio.cam = new vk::PinholeCamera(64, 64, 1., 64., 64., 32., 32.);
    vio.grid_size = 16;
    vio.patch_size = 8;
    vio.patch_pyrimid_level = 1;
    // One real iteration isolates each construction from later correspondence
    // changes and gives identical inputs to both feedback and baseline builds.
    vio.max_iterations = 1;
    vio.img_point_cov = 100.;
    vio.normal_en = false;
    vio.inverse_composition_en = inverse;
    vio.exposure_estimate_en = true;
    vio.raycast_en = false;
    vio.Rli.setIdentity();
    vio.Rcl.setIdentity();
    vio.Pli.setZero();
    vio.Pcl.setZero();
    vio.G.setZero();
    vio.H_T_H.setZero();
    vio.initializeVIO();
  }

  void select(const cv::Mat &current, const cv::Mat &reference, bool supported)
  {
    vio.visual_submap->reset();
    point.reset();
    // Each construction starts at the center projection, including a supported
    // call after an unsupported one. The same manager/tracker object is reused.
    state = StatesGroup();
    state.cov.setIdentity();
    state.cov *= 0.01;
    propagated = state;
    vio.new_frame_.reset(new Frame(vio.cam, current));
    vio.updateFrameState(state);
    vio.total_points = supported ? 1 : 0;
    if (!supported) return;

    point.reset(new VisualPoint(V3D(0., 0., 2.)));
    const V2D pixel(32., 32.);
    std::vector<float> patch(vio.patch_size_total);
    vio.getImagePatch(reference, pixel, patch.data(), 0);
    float *owned_patch = new float[patch.size()];
    std::copy(patch.begin(), patch.end(), owned_patch);
    Feature *feature = new Feature(point.get(), owned_patch, pixel,
      vio.cam->cam2world(pixel), SE3(M3D::Identity(), V3D::Zero()), 0);
    feature->img_ = reference;
    feature->id_ = 0;
    feature->inv_expo_time_ = 1.;
    point->addFrameRef(feature); // The point owns this reference feature.
    point->ref_patch = feature;
    point->has_ref_patch_ = true;
    vio.visual_submap->voxel_points.push_back(point.get());
    vio.visual_submap->warp_patch.push_back(patch);
    vio.visual_submap->search_levels.push_back(0);
    vio.visual_submap->inv_expo_list.push_back(1.);
    vio.visual_submap->errors.push_back(0.);
    vio.visual_submap->propa_errors.push_back(0.);
  }

  bool legacyRetained() const
  {
    return vio.diag_accepted > vio.diag_reverted && vio.diag_pixels > 0 &&
      vio.diag_nonfinite == 0 && finiteState(state);
  }

  StatesGroup run(const std::string &name, const cv::Mat &current)
  {
    const StatesGroup before = state;
    vio.diag_accepted = vio.diag_reverted = vio.diag_pixels = vio.diag_nonfinite = 0;
    vio.diag_last_mse = 0.;
    // processFrame normally owns these call boundaries. This fixture bypasses
    // retrieval only; linearization/accept/rollback/commit hooks are all real.
    vio.visual_feedback_tracker.reset();
    vio.computeJacobianAndUpdateEKF(current);
    vio.visual_feedback_tracker.finish(finiteState(state));
    require(finiteState(state), name + ": finite filter state");
    emit(name, before);
    return before;
  }

  void emit(const std::string &name, const StatesGroup &before) const
  {
    std::cout << "FILTER {\"case\":\"" << name << "\",\"R\":";
    matrixJson(state.rot_end);
    std::cout << ",\"p\":"; matrixJson(state.pos_end);
    std::cout << ",\"v\":"; matrixJson(state.vel_end);
    std::cout << ",\"bg\":"; matrixJson(state.bias_g);
    std::cout << ",\"ba\":"; matrixJson(state.bias_a);
    std::cout << ",\"g\":"; matrixJson(state.gravity);
    std::cout << ",\"inv_exposure\":" << state.inv_expo_time;
    std::cout << ",\"P\":"; matrixJson(state.cov);
    std::cout << ",\"G\":"; matrixJson(vio.G);
    std::cout << ",\"H_T_H\":"; matrixJson(vio.H_T_H);
    std::cout << ",\"mean_max_delta\":" << meanDelta(state, before)
      << ",\"cov_max_delta\":" << (state.cov - before.cov).cwiseAbs().maxCoeff()
      << ",\"accepted\":" << vio.diag_accepted
      << ",\"reverted\":" << vio.diag_reverted
      << ",\"pixels\":" << vio.diag_pixels
      << ",\"nonfinite\":" << vio.diag_nonfinite
      << ",\"mse\":" << vio.diag_last_mse
      << ",\"legacy_retained\":" << legacyRetained() << "}\n";
    const auto &feedback = vio.visual_feedback_tracker.result();
    std::cout << "FEEDBACK {\"case\":\"" << name
      << "\",\"eligible\":" << feedback.eligible(legacyRetained())
      << ",\"pose_information\":" << feedback.path.pose_information
      << ",\"exposure_information\":" << feedback.path.exposure_information
      << ",\"surviving_accepts\":" << feedback.path.surviving_accepts
      << ",\"attempts\":" << feedback.attempted_linearizations
      << ",\"covariance_committed\":" << feedback.covariance_committed
      << ",\"reason\":\"" << feedback.reason(legacyRetained()) << "\"}\n";
  }

  void checkFeedback(bool supported, bool pose, bool exposure) const
  {
    const auto &r = vio.visual_feedback_tracker.result();
    require(r.executed && r.completed && r.final_finite, "completed finite call");
    require(!r.nonfinite_attempt && !r.invalid_measurement_shape, "valid solver inputs");
    require(r.attempted_linearizations == (supported ? 1U : 0U), "fresh attempt count");
    require(r.accepted_attempts == (supported ? 1U : 0U), "fresh accepted count");
    require(r.path.surviving_accepts == (supported ? 1U : 0U), "fresh retained path");
    require(r.last_measurement_rows == (supported ? 64U : 0U), "actual measurement rows");
    require(r.path.pose_information == pose, "actual pose information");
    require(r.path.exposure_information == exposure, "actual exposure information");
    require(r.covariance_committed == supported, "actual covariance commit");
    require(r.eligible(legacyRetained()) == (supported && pose), "commit eligibility");
  }
};

void supportedConstruction(bool inverse, const std::string &kind)
{
  const std::string name = (inverse ? "inverse_" : "forward_") + kind;
  Fixture f(inverse);
  cv::Mat current;
  cv::Mat reference;
  const bool textured = kind == "zero_innovation" || kind == "nonzero_innovation";
  if (kind == "zero_H")
  {
    current = image(false, 0);
    reference = image(false, 32); // Nonzero residual does not make H informative.
  }
  else if (kind == "exposure_only")
  {
    current = image(false, 100);
    reference = image(false, 102);
  }
  else
  {
    current = image(true);
    reference = current.clone();
    if (kind == "nonzero_innovation") reference += cv::Scalar(2);
  }
  f.select(current, reference, true);
  const StatesGroup before = f.run(name, current);
  require(f.legacyRetained(), name + ": accepted finite production branch");
  require(f.vio.diag_accepted == 1 && f.vio.diag_reverted == 0 &&
    f.vio.diag_pixels == 64, name + ": one real accepted iteration");
  const double pose_gram = f.vio.H_T_H.block<6, 6>(0, 0).cwiseAbs().maxCoeff();
  const double covariance_delta = (f.state.cov - before.cov).cwiseAbs().maxCoeff();
  require((pose_gram > 0.) == textured, name + ": actual pose Gram classification");
  const Eigen::Matrix<double, DIM_STATE, DIM_STATE> expected =
    before.cov - f.vio.G * before.cov;
  require((f.state.cov - expected).cwiseAbs().maxCoeff() < 1e-14,
    name + ": actual covariance commitment");
  if (kind == "zero_H")
  {
    require(f.vio.H_T_H.isZero(0.) && f.vio.G.isZero(0.), name + ": exact zero H/G");
    require(meanDelta(f.state, before) == 0. && covariance_delta == 0.,
      name + ": exact unchanged mean/P despite a nonzero residual");
    require(f.vio.diag_last_mse > 0., name + ": nonzero residual witness");
  }
  else
  {
    require(covariance_delta > 0., name + ": P changes");
    if (kind == "zero_innovation")
      require(meanDelta(f.state, before) == 0. && f.vio.diag_last_mse == 0.,
        name + ": zero innovation has zero mean change, yet changes P");
    else
      require(meanDelta(f.state, before) > 0. && f.vio.diag_last_mse > 0.,
        name + ": nonzero innovation changes the mean");
    if (kind == "exposure_only")
    {
      require(f.vio.H_T_H(6, 6) > 0., name + ": exposure column exists");
      require((f.state.rot_end - before.rot_end).isZero(0.) &&
        (f.state.pos_end - before.pos_end).isZero(0.), name + ": pose stays fixed");
    }
  }
  f.checkFeedback(true, textured, !inverse && kind != "zero_H");
}

void resetSequence(bool inverse)
{
  Fixture f(inverse);
  const cv::Mat current = image(true);
  const std::string prefix = inverse ? "inverse_reset_" : "forward_reset_";
  // Exercise cold absence, new success, absence after success, and new success
  // using the same tracker. Stale solver matrices must not become new evidence.
  for (int index = 0; index < 4; ++index)
  {
    const bool supported = index % 2 == 1;
    f.select(current, current, supported);
    const StatesGroup before = f.run(prefix + std::to_string(index), current);
    f.checkFeedback(supported, supported, supported && !inverse);
    require(f.legacyRetained() == supported, prefix + ": fresh legacy diagnostics");
    if (!supported)
      require(meanDelta(f.state, before) == 0. &&
        (f.state.cov - before.cov).isZero(0.), prefix + ": unsupported call is unchanged");
  }
}

void informationThenZeroH()
{
  Fixture f(false);
  const cv::Mat textured = image(true);
  cv::Mat reference = textured.clone();
  reference += cv::Scalar(2);
  f.select(textured, reference, true);
  const StatesGroup before = f.state;
  f.vio.diag_accepted = f.vio.diag_reverted = f.vio.diag_pixels = f.vio.diag_nonfinite = 0;
  f.vio.diag_last_mse = 0.;
  f.vio.compute_jacobian_time = f.vio.update_ekf_time = 0.;
  f.vio.visual_feedback_tracker.reset();
  // A bounded, deliberately constructed level driver, not a claim that a real
  // image becomes black between production levels. Both updates use the actual
  // production equation: each accepted estimate is relative to state_propagat.
  f.vio.updateState(textured, 0);
  require(finiteState(f.state) && meanDelta(f.state, before) > 0.,
    "information_then_zero_H: first actual update changes mean");
  require(f.vio.H_T_H.block<6, 6>(0, 0).cwiseAbs().maxCoeff() > 0.,
    "information_then_zero_H: first actual update has pose information");
  require(f.vio.visual_feedback_tracker.result().path.pose_information,
    "information_then_zero_H: first retained estimate is informative");
  // This drives the second real updateState and the real covariance-commit
  // hook, with no tracker reset or intervening covariance commit. H=G=0 makes
  // the second accepted estimate return to the propagated mean, superseding
  // the preceding informative estimate rather than accumulating its evidence.
  f.vio.computeJacobianAndUpdateEKF(image(false, 0));
  f.vio.visual_feedback_tracker.finish(finiteState(f.state));
  require(finiteState(f.state) && f.legacyRetained() &&
    f.vio.diag_accepted == 2 && f.vio.diag_reverted == 0,
    "information_then_zero_H: both production updates are accepted and finite");
  require(f.vio.H_T_H.isZero(0.) && f.vio.G.isZero(0.),
    "information_then_zero_H: final actual H and G are exactly zero");
  require(meanDelta(f.state, before) < 1e-12 &&
    (f.state.cov - before.cov).isZero(0.),
    "information_then_zero_H: final mean returns to prior and P is unchanged");
  const auto &result = f.vio.visual_feedback_tracker.result();
  require(result.attempted_linearizations == 2 && result.accepted_attempts == 2 &&
    !result.nonfinite_attempt && result.covariance_committed && result.covariance_finite,
    "information_then_zero_H: two real accepts and a finite covariance commit");
  require(!result.path.pose_information && !result.path.exposure_information &&
    !result.eligible(f.legacyRetained()),
    "information_then_zero_H: superseded information cannot confer eligibility");
  f.emit("forward_information_then_zero_H", before);
}
} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(17);
    for (bool inverse : {false, true})
    {
      supportedConstruction(inverse, "zero_H");
      supportedConstruction(inverse, "zero_innovation");
      supportedConstruction(inverse, "nonzero_innovation");
      resetSequence(inverse);
    }
    supportedConstruction(false, "exposure_only");
    informationThenZeroH();
    std::cout << "PASS actual VIO solver constructions; functional evidence only.\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "FAIL actual VIO solver construction: " << error.what() << '\n';
    return 1;
  }
}
