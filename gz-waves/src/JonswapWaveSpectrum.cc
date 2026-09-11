// Copyright (C) 2025  AquaFind
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "gz/waves/JonswapWaveSpectrum.hh"

#include <algorithm>
#include <cmath>

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
JonswapWaveSpectrum::~JonswapWaveSpectrum()
{
}

//////////////////////////////////////////////////
JonswapWaveSpectrum::JonswapWaveSpectrum(
    double hs, double tp, double gamma, double gravity) :
  OmniDirectionalWaveSpectrum(),
  gravity_(gravity),
  hs_(hs),
  tp_(tp),
  gamma_(gamma)
{
  RecalcNormalisation();
}

//////////////////////////////////////////////////
double JonswapWaveSpectrum::EvaluateOmega(double omega) const
{
  if (omega <= 0.0 || std::abs(tp_) < 1.0E-8)
  {
    return 0.0;
  }

  const double omega_p = 2.0 * M_PI / tp_;
  const double sigma = (omega <= omega_p) ? 0.07 : 0.09;

  const double r = std::exp(
      -std::pow(omega - omega_p, 2.0) /
      (2.0 * sigma * sigma * omega_p * omega_p));

  const double peak_shape = std::exp(
      -1.25 * std::pow(omega_p / omega, 4.0));

  const double g2 = gravity_ * gravity_;
  const double omega5 = std::pow(omega, 5.0);

  return (g2 / omega5) * peak_shape * std::pow(gamma_, r);
}

//////////////////////////////////////////////////
void JonswapWaveSpectrum::RecalcNormalisation()
{
  if (std::abs(hs_) < 1.0E-8 || std::abs(tp_) < 1.0E-8)
  {
    norm_scale_ = 0.0;
    return;
  }

  const double omega_p = 2.0 * M_PI / tp_;

  // Integrate the unnormalised spectrum over a fixed range relative to the
  // peak frequency using the trapezoidal rule.
  const double omega_min = 0.05 * omega_p;
  const double omega_max = 10.0 * omega_p;
  const int n = 4000;
  const double d_omega = (omega_max - omega_min) / n;

  double m0 = 0.0;
  double s_prev = EvaluateOmega(omega_min);
  for (int i = 1; i <= n; ++i)
  {
    double omega = omega_min + i * d_omega;
    double s_curr = EvaluateOmega(omega);
    m0 += 0.5 * (s_prev + s_curr) * d_omega;
    s_prev = s_curr;
  }

  if (m0 < 1.0E-12)
  {
    norm_scale_ = 0.0;
    return;
  }

  // Choose norm_scale_ so that 4 * sqrt(norm_scale_ * m0) == hs_.
  norm_scale_ = (hs_ * hs_ / 16.0) / m0;
}

//////////////////////////////////////////////////
double JonswapWaveSpectrum::Evaluate(double k) const
{
  if (std::abs(k) < 1.0E-8 || std::abs(norm_scale_) < 1.0E-12)
  {
    return 0.0;
  }

  // deep water dispersion relation and its derivative
  const double omega = std::sqrt(gravity_ * k);
  const double domega_dk = 0.5 * std::sqrt(gravity_ / k);

  return norm_scale_ * EvaluateOmega(omega) * domega_dk;
}

//////////////////////////////////////////////////
void JonswapWaveSpectrum::Evaluate(
    Eigen::Ref<Eigen::ArrayXXd> spectrum,
    const Eigen::Ref<const Eigen::ArrayXXd>& k) const
{
  /// \note Eigen asserts cbegin and cend are from the same expression.
  auto k_view = k.reshaped();
  std::transform(
    k_view.cbegin(),
    k_view.cend(),
    spectrum.reshaped().begin(),
    [this] (double k_i) -> double
    {
      return this->Evaluate(k_i);
    });
}

//////////////////////////////////////////////////
double JonswapWaveSpectrum::Gravity() const
{
  return gravity_;
}

//////////////////////////////////////////////////
void JonswapWaveSpectrum::SetGravity(double value)
{
  gravity_ = value;
  RecalcNormalisation();
}

//////////////////////////////////////////////////
double JonswapWaveSpectrum::Hs() const
{
  return hs_;
}

//////////////////////////////////////////////////
void JonswapWaveSpectrum::SetHs(double value)
{
  hs_ = value;
  RecalcNormalisation();
}

//////////////////////////////////////////////////
double JonswapWaveSpectrum::Tp() const
{
  return tp_;
}

//////////////////////////////////////////////////
void JonswapWaveSpectrum::SetTp(double value)
{
  tp_ = value;
  RecalcNormalisation();
}

//////////////////////////////////////////////////
double JonswapWaveSpectrum::Gamma() const
{
  return gamma_;
}

//////////////////////////////////////////////////
void JonswapWaveSpectrum::SetGamma(double value)
{
  gamma_ = value;
  RecalcNormalisation();
}

}  // namespace waves
}  // namespace gz
