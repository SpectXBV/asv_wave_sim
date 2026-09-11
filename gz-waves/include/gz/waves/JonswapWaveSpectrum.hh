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

/// \file JonswapWaveSpectrum.hh
///
/// \brief A JONSWAP omnidirectional wave spectrum parameterised directly by
///        significant wave height (Hs), peak period (Tp) and the JONSWAP
///        peak-enhancement factor (gamma), rather than by wind speed.
///
/// This is a standalone addition alongside the existing spectra in
/// WaveSpectrum.hh (PiersonMoskowitzWaveSpectrum, ECKVWaveSpectrum) and does
/// not modify that file. It implements the same OmniDirectionalWaveSpectrum
/// interface so it can be dropped into the same spectrum + spreading-function
/// pattern used by the FFT wave simulation.

#ifndef GZ_WAVES_JONSWAPWAVESPECTRUM_HH_
#define GZ_WAVES_JONSWAPWAVESPECTRUM_HH_

#include "gz/waves/WaveSpectrum.hh"

namespace gz
{
namespace waves
{
/// \brief JONSWAP omnidirectional wave spectrum.
///
/// Unlike ECKVWaveSpectrum / PiersonMoskowitzWaveSpectrum (both driven by
/// wind speed, with Hs and Tp coupled through the wind-sea relation), this
/// spectrum takes Hs and Tp as independent inputs, so it can be used to seed
/// the wave field with a known ground-truth sea state.
class JonswapWaveSpectrum : public OmniDirectionalWaveSpectrum
{
 public:
  virtual ~JonswapWaveSpectrum();

  /// \brief Construct a JONSWAP spectrum.
  ///
  /// \param[in] hs      Significant wave height (m).
  /// \param[in] tp      Peak period (s).
  /// \param[in] gamma   Peak-enhancement factor (standard JONSWAP: 3.3).
  /// \param[in] gravity Gravitational acceleration (m/s^2).
  explicit JonswapWaveSpectrum(
      double hs = 2.0,
      double tp = 8.0,
      double gamma = 3.3,
      double gravity = 9.81);

  double Evaluate(double k) const override;

  void Evaluate(
      Eigen::Ref<Eigen::ArrayXXd> spectrum,
      const Eigen::Ref<const Eigen::ArrayXXd>& k) const override;

  double Gravity() const;

  void SetGravity(double value);

  double Hs() const;

  void SetHs(double value);

  double Tp() const;

  void SetTp(double value);

  double Gamma() const;

  void SetGamma(double value);

 private:
  /// \brief Recompute the Hs-normalisation scale factor. Must be called
  ///        whenever hs_, tp_, gamma_ or gravity_ change.
  void RecalcNormalisation();

  /// \brief Unnormalised JONSWAP spectral density in angular frequency
  ///        (omega) space.
  double EvaluateOmega(double omega) const;

  double gravity_{9.81};
  double hs_{2.0};
  double tp_{8.0};
  double gamma_{3.3};

  /// \brief Scale factor applied to the unnormalised spectrum so that
  ///        4 * sqrt(m0) == hs_, where m0 = integral of S(omega) domega.
  ///        Precomputed by numerical integration in RecalcNormalisation().
  double norm_scale_{1.0};
};
}  // namespace waves
}  // namespace gz

#endif  // GZ_WAVES_JONSWAPWAVESPECTRUM_HH_
