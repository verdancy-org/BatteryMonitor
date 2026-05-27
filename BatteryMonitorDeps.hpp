#pragma once

#include "libxr_def.hpp"

#include <algorithm>
#include <cmath>

#if __has_include("LPFilter.hpp")
#include "LPFilter.hpp"
#else
template <typename T = LIBXR_DEFAULT_SCALAR,
          typename Scalar = LIBXR_DEFAULT_SCALAR>
class LPFilter
{
 public:
  LPFilter() = default;

  LPFilter(Scalar sample_interval, Scalar time_constant)
  {
    SetParameters(sample_interval, time_constant);
  }

  explicit LPFilter(Scalar time_constant) : time_constant_(time_constant) {}

  static T Calculate(const T& current, const T& sample, Scalar alpha)
  {
    alpha = SanitizeAlpha(alpha);
    return current + alpha * (sample - current);
  }

  void SetParameters(Scalar sample_interval, Scalar time_constant)
  {
    if (!std::isfinite(sample_interval) || !std::isfinite(time_constant) ||
        sample_interval <= Scalar(0) || time_constant < Scalar(0))
    {
      return;
    }

    const Scalar denominator = time_constant + sample_interval;
    if (denominator > SIGMA_DEF)
    {
      SetAlpha(sample_interval / denominator);
    }

    time_constant_ = time_constant;
  }

  bool SetCutoffFrequency(Scalar sample_frequency, Scalar cutoff_frequency)
  {
    if (!std::isfinite(sample_frequency) || !std::isfinite(cutoff_frequency) ||
        sample_frequency <= Scalar(0) || cutoff_frequency <= Scalar(0) ||
        cutoff_frequency >= sample_frequency / Scalar(2))
    {
      return false;
    }

    SetParameters(
        Scalar(1) / sample_frequency,
        Scalar(1) /
            (static_cast<Scalar>(LibXR::TWO_PI) * cutoff_frequency));
    return true;
  }

  void SetCutoffFrequency(Scalar cutoff_frequency)
  {
    if (std::isfinite(cutoff_frequency) && cutoff_frequency > SIGMA_DEF)
    {
      time_constant_ =
          Scalar(1) /
          (static_cast<Scalar>(LibXR::TWO_PI) * cutoff_frequency);
    }
    else
    {
      time_constant_ = Scalar(0);
    }
  }

  void SetAlpha(Scalar alpha) { alpha_ = SanitizeAlpha(alpha); }

  void Reset(const T& sample = T{}) { state_ = sample; }

  const T& Update(const T& sample)
  {
    state_ = Calculate(state_, sample, alpha_);
    return state_;
  }

  const T& Update(const T& sample, Scalar dt)
  {
    if (!std::isfinite(dt) || dt <= Scalar(0))
    {
      return state_;
    }

    SetParameters(dt, time_constant_);
    return Update(sample);
  }

  const T& State() const { return state_; }

 private:
  static Scalar SanitizeAlpha(Scalar alpha)
  {
    if (!std::isfinite(alpha))
    {
      return Scalar(0);
    }

    return std::clamp(alpha, Scalar(0), Scalar(1));
  }

  static constexpr Scalar SIGMA_DEF = static_cast<Scalar>(1e-6);

  Scalar time_constant_ = Scalar(0);
  Scalar alpha_ = Scalar(0);
  T state_{};
};
#endif
